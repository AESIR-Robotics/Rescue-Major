#pragma once
// =============================================================================
// can_iface_manager.hpp
//
// Gestor compartido de la interfaz CAN física.
// Múltiples instancias de CAN_Transport sobre la misma interfaz
// comparten un CANIfaceManager via shared_ptr — solo la primera
// instancia que la necesite la configura, y solo la última la baja.
//
// Uso:
//   auto mgr = std::make_shared<CANIfaceManager>("can1", 500000);
//   arm_0.init(mgr, ...);
//   arm_1.init(mgr, ...);
//   // mgr->ifDown() solo ocurre cuando ambas instancias se destruyen
// =============================================================================

#include <atomic>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>
#include <net/if.h> 
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>

#include "utils/logger.hpp"
#include "utils/diagnostics.hpp"

// This should be in its own util with runCmd
class ChildGuard {
public:
    explicit ChildGuard(pid_t pid) : pid_(pid) {}

    ~ChildGuard() {
        if (pid_ <= 0) return;

        int status = 0;
        pid_t r = waitpid(pid_, &status, WNOHANG);

        if (r == pid_) {
            return;
        }

        if (r == 0) {
            kill(pid_, SIGKILL);
            // reap seguro
            while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR);
            return;
        }

        if (r < 0) {
            if (errno == ECHILD) {
                return;
            }

            kill(pid_, SIGKILL);
            while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR);
        }
    }

    void release() {
        pid_ = -1; // ya no soy responsable
    }

    pid_t pid() const { return pid_; }

private:
    pid_t pid_;
};

class CANIfaceManager {
public:
    // ── Estado de la interfaz ─────────────────────────────────────────────────
    enum class IfaceState {
        UNKNOWN,
        UP_CONFIGURED,    ///< up y bitrate correcto — no hace falta nada
        UP_WRONG_BITRATE, ///< up pero bitrate diferente — necesita reconfig
        DOWN,             ///< existe pero está down
        NOT_FOUND,        ///< la interfaz no existe en el sistema
    };

    explicit CANIfaceManager(DiagnosticRegistry *reg = nullptr, std::string  iface = "",
                             uint32_t     bitrate    = 500000,
                             uint32_t     restart_ms = 100)
        : iface_{std::move(iface)},
          bitrate_{bitrate},
          restart_ms_{restart_ms}
    {
        statusReport.with([this](Status &d){
            d.hardware_id = iface_;
            d.level = Status::OK;
            d.message = "The status of the kernel CAN interface";
            d.name = "CAN_Interface";
            d.values.emplace(std::make_pair("connected", "false"));
            d.values.emplace(std::make_pair("instances", "0"));
            d.values.emplace(std::make_pair("perms", "able"));
            d.values.emplace(std::make_pair("con_attmps", "0"));
        });
        if(reg){
            reg->register_source(&statusReport);
        }
    }

    ~CANIfaceManager() {
        // Por si acaso quedan refs — no hacer down si hay instancias activas
        if (ref_count_.load() == 0 && brought_up_)
            ifDown();
    }

    CANIfaceManager(const CANIfaceManager &)            = delete;
    CANIfaceManager &operator=(const CANIfaceManager &) = delete;

    void setLogger(Logger &in_log) {
        log = in_log;
    }

    // ── acquire — llamado por cada CAN_Transport al conectarse ────────────────
    // Primera llamada: evalúa el estado de la interfaz y la configura si hace falta.
    // Llamadas siguientes: solo incrementan ref_count si la interfaz está bien.
    // Retorna true si la interfaz está up y lista para abrir sockets.
    bool acquire() {
        std::lock_guard<std::mutex> lk(mutex_);

        IfaceState state = checkState();

        switch (state) {
        case IfaceState::NOT_FOUND:
            log.logError("CANIfaceManager: interface %s not found", iface_.c_str());
            statusReport.with([](Status &d){
                d.level = Status::ERROR;
                d.values["connected"] = "false";
            });
            return false;

        case IfaceState::UP_CONFIGURED:
            // Ya está bien — solo incrementar
            log.logInfo("CANIfaceManager: %s already up at %u bps", iface_.c_str(), bitrate_);
            ++ref_count_;
            statusReport.with([this](Status &d){
                d.level = Status::OK;
                d.values["connected"] = "true";
                d.values["instances"] = std::to_string(ref_count_);
            });
            return true;

        case IfaceState::UP_WRONG_BITRATE:
            // Está up pero con bitrate diferente — necesita reconfig
            log.logWarn("CANIfaceManager: %s up but wrong bitrate — reconfiguring", iface_.c_str());
            if (!doDown()) return false;
            if (!doConfigure()) {}
            if (!doUp()) return false;
            brought_up_ = true;
            ++ref_count_;
            statusReport.with([this](Status &d){
                d.values["instances"] = std::to_string(ref_count_);
            });
            return true;

        case IfaceState::DOWN:
        case IfaceState::UNKNOWN:
            statusReport.with([](Status &d){
                d.level = Status::OK;
                d.values["connected"] = "false";
            });
            // Está down o estado desconocido — configurar y subir
            log.logInfo("CANIfaceManager: bringing up %s at %u bps", iface_.c_str(), bitrate_);
            if (!doConfigure()) {};
            if (!doUp()) return false;
            brought_up_ = true;
            ++ref_count_;
            statusReport.with([this](Status &d){
                d.values["instances"] = std::to_string(ref_count_);
            });
            return true;
        }
        statusReport.with([](Status &d){
            d.level = Status::ERROR;
            d.values["connected"] = "false";
        });
        return false;
    }

    // ── release — llamado por cada CAN_Transport al desconectarse ─────────────
    // Cuando ref_count llega a 0 y la interfaz fue levantada por nosotros, la baja.
    void release() {
        std::lock_guard<std::mutex> lk(mutex_);
        
        if (--ref_count_ == 0 && brought_up_) {
            log.logInfo("CANIfaceManager: last user released — bringing down %s", iface_.c_str());
            doDown();
            brought_up_ = false;
        }

        statusReport.with([this](Status &d){
            d.values["instances"] = std::to_string(ref_count_);
        });
        
    }

    const std::string &iface()      const { return iface_;   }
    uint32_t           bitrate()    const { return bitrate_;  }
    int                refCount()   const { return ref_count_.load(); }

private:
    // ── Estado de la interfaz — sin fork ──────────────────────────────────────
    // Lee /sys/class/net/<iface>/operstate y /sys/class/net/<iface>/can/bit_rate
    IfaceState checkState() const {
        // Verificar existencia via if_nametoindex (syscall directa, sin fork)
        if (if_nametoindex(iface_.c_str()) == 0)
            return IfaceState::NOT_FOUND;

        // Leer operstate
        std::string operstate = readSysfs("operstate");
        bool is_up = (operstate == "up");

        // Leer bitrate actual
        // El kernel expone esto en /sys/class/net/<iface>/can/bit_rate (u32 en texto)
        std::string bitrate_str = readSysfs("can/bit_rate");
        uint32_t current_bitrate = 0;
        (void)current_bitrate;
        if (!bitrate_str.empty()) {
            try { current_bitrate = static_cast<uint32_t>(std::stoul(bitrate_str)); }
            catch (...) { current_bitrate = 0; }
        }

        if (!is_up) return IfaceState::DOWN;
        //if (current_bitrate != bitrate_) return IfaceState::UP_WRONG_BITRATE;
        return IfaceState::UP_CONFIGURED;
    }

    std::string readSysfs(const std::string &attr) const {
        std::string path = "/sys/class/net/" + iface_ + "/" + attr;
        std::ifstream f(path);
        if (!f) return "";
        std::string val;
        std::getline(f, val);
        // Trim trailing whitespace
        while (!val.empty() && (val.back() == '\n' || val.back() == '\r' || val.back() == ' '))
            val.pop_back();
        return val;
    }

    // ── Operaciones de interfaz ───────────────────────────────────────────────
    bool doDown() {
        std::string cmd = "sudo -n ip link set " + iface_ + " down";
        
        if (!runCmd(cmd.c_str())) {
            log.logError("CANIfaceManager: failed to bring down %s", iface_.c_str());
            statusReport.with([](Status &d){
                d.level = Status::ERROR;
                d.values["perms"] = "unable";
            });
            return false;
        }
        statusReport.with([](Status &d){
            d.level = Status::OK;
            d.values["connected"] = "false";
            d.values["perms"] = "able";
        });
        return true;
    }

    bool doConfigure() {
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
            "sudo -n ip link set %s type can bitrate %u restart-ms %u",
            iface_.c_str(), bitrate_, restart_ms_);
        if (!runCmd(cmd)) {
            log.logError("CANIfaceManager: failed to configure %s", iface_.c_str());
            return false;
        }
        return true;
    }

    bool doUp() {
        attmps++;
        statusReport.with([this](Status &d){
            d.values["con_attmps"] = std::to_string(attmps);
        });

        std::string cmd = "sudo -n ip link set " + iface_ + " up";
        if (!runCmd(cmd.c_str())) {
            log.logError("CANIfaceManager: failed to bring up %s", iface_.c_str());
            statusReport.with([](Status &d){
                d.level = Status::ERROR;
                d.values["perms"] = "unable";
            });
            return false;
        }
        statusReport.with([](Status &d){
            d.level = Status::OK;
            d.values["connected"] = "true";
            d.values["perms"] = "able";
        });
        return true;
    }

    void ifDown() {
        std::string cmd = "sudo -n ip link set " + iface_ + " down";
        runCmd(cmd.c_str());  // best-effort
        log.logInfo("CANIfaceManager: %s down", iface_.c_str());
        statusReport.with([](Status &d){
            d.values["connected"] = "false";
        });
    }

    // ── fork/exec sin system() ────────────────────────────────────────────────
    static bool runCmd(const char *cmd) {
        std::string buf(cmd);
        std::vector<char *> argv;
        char *tok = &buf[0];
        char *end = tok + buf.size();
        while (tok < end) {
            while (tok < end && *tok == ' ') ++tok;
            if (tok >= end) break;
            argv.push_back(tok);
            while (tok < end && *tok != ' ') ++tok;
            if (tok < end) *tok++ = '\0';
        }
        if (argv.empty()) return false;
        argv.push_back(nullptr);

        ChildGuard guard{fork()};
        pid_t pid = guard.pid();
        if (pid < 0) return false;
        if (pid == 0) {
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDOUT_FILENO);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            execvp(argv[0], argv.data());
            _exit(127);
        }

        
        constexpr int timeout_ms = 500;
        constexpr int poll_ms    = 10;
        int waited = 0;
        int status = 0;
        while (waited < timeout_ms) {
            pid_t r = waitpid(pid, &status, WNOHANG);

            if (r == pid) {
                guard.release();
                break;
            }

            if (r < 0) {
                if (errno == EINTR) continue;
                return false;
            }

            struct timespec ts{ 0, poll_ms * 1'000'000L };
            while (nanosleep(&ts, &ts) < 0 && errno == EINTR);

            waited += poll_ms;
        }

        if (waited >= timeout_ms) {
            return false; // destructor mata al hijo
        }

        return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    }

    // ── Miembros ──────────────────────────────────────────────────────────────
    std::string       iface_;
    uint32_t          bitrate_;
    uint32_t          restart_ms_;
    std::atomic<int>  ref_count_{ 0 };
    bool              brought_up_{ false };
    std::mutex        mutex_;
    Tracked<Status> statusReport;
    u_int32_t  attmps { 0 };
    Logger            log;
};