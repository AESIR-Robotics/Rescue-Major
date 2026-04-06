#pragma once
// =============================================================================
// serial_mux.hpp
//
// Multiplexer local sobre stream serial. Replica el modelo del kernel CAN:
//   - Un hilo lector parsea frames [CAN_ID 4B][frame...] y los deposita
//     en el ring del canal registrado con ese rx_id.
//   - TX protegido por mutex — escritura atómica por instancia.
//   - Reconexión automática en background con backoff exponencial.
//   - Ownership vía shared_ptr — el mux vive mientras algún transport lo use.
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>
#include <cerrno>

using LogFn = std::function<void(const std::string &)>;

// =============================================================================
// MuxRing — SPSC lock-free ring buffer
// Productor: hilo lector del SerialMux
// Consumidor: Bridge_Transport::readData en el tick del nodo
// =============================================================================
template <size_t SIZE>
class MuxRing {
public:
    bool push(uint8_t b) {
        size_t next = (head_.load(std::memory_order_relaxed) + 1) % SIZE;
        if (next == tail_.load(std::memory_order_acquire)) return false;
        buf_[head_.load(std::memory_order_relaxed)] = b;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(uint8_t &b) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        b = buf_[t];
        tail_.store((t + 1) % SIZE, std::memory_order_release);
        return true;
    }

    bool   empty()     const { return available() == 0; }
    size_t available() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (h + SIZE - t) % SIZE;
    }

    void reset() {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }

private:
    std::atomic<size_t> head_{0}, tail_{0};
    uint8_t buf_[SIZE]{};
};

// =============================================================================
// SerialMux
// =============================================================================
class SerialMux {
public:
    static constexpr size_t CAN_ID_BYTES    = 4;
    static constexpr size_t MAX_PAYLOAD     = 60;
    static constexpr size_t MAX_FRAME_BYTES = MAX_PAYLOAD + 4;
    static constexpr size_t RING_SIZE       = 512;

    struct Channel {
        uint32_t           rx_id{0};
        MuxRing<RING_SIZE> ring{};
    };

    SerialMux()  = default;
    ~SerialMux() { stop(); }

    SerialMux(const SerialMux &)            = delete;
    SerialMux &operator=(const SerialMux &) = delete;

    // ── Init ─────────────────────────────────────────────────────────────────
    bool init(const std::string &port, uint32_t baud,
              LogFn info = {}, LogFn warn = {}, LogFn error = {}) {
        port_      = port;
        baud_      = baud;
        log_info_  = std::move(info);
        log_warn_  = std::move(warn);
        log_error_ = std::move(error);
        return openSerial();
    }

    // ── Registro de canales — llamar ANTES de startReader() ──────────────────
    Channel *registerChannel(uint32_t rx_id) {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        channels_.push_back(std::make_unique<Channel>());
        channels_.back()->rx_id = rx_id;
        return channels_.back().get();
    }

    // ── Arrancar hilo lector — llamar DESPUÉS de registrar todos los canales ─
    void startReader() {
        running_.store(true);
        reader_thread_ = std::thread(&SerialMux::readerLoop, this);
    }

    void stop() {
        running_.store(false);
        if (reader_thread_.joinable()) reader_thread_.join();
        closeFd();
    }

    // ── Reconexión manual — Bridge_Transport::reconnect() la llama ───────────
    // Si ya está conectado retorna true inmediatamente.
    // Si no, intenta una reconexión síncrona (el hilo también lo hace en BG).
    bool reconnect() {
        if (connected()) return true;
        return openSerial();
    }

    bool connected() const { return connected_.load(std::memory_order_acquire); }

    // ── TX — escritura atómica ────────────────────────────────────────────────
    size_t write(const uint8_t *data, size_t length) {
        if (!connected() || !data || length == 0) return 0;
        std::lock_guard<std::mutex> lk(tx_mutex_);
        ssize_t sent = ::write(fd_, data, length);
        if (sent < 0) {
            logE("SerialMux write() failed: %s", strerror(errno));
            return 0;
        }
        return static_cast<size_t>(sent);
    }

    // Expuesto para que Bridge_Transport arme el frame TX
    static void toLE(uint32_t id, uint8_t b[4]) {
        b[0]=id&0xFF; b[1]=(id>>8)&0xFF;
        b[2]=(id>>16)&0xFF; b[3]=(id>>24)&0xFF;
    }

private:
    // ── Apertura del puerto ───────────────────────────────────────────────────
    bool openSerial() {
        closeFd();

        int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            logE("SerialMux open(%s) failed: %s", port_.c_str(), strerror(errno));
            return false;
        }
        if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
            logE("SerialMux flock() failed: %s", strerror(errno));
            ::close(fd); return false;
        }
        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
        cfmakeraw(&tty);
        cfsetispeed(&tty, baudToSpeed(baud_));
        cfsetospeed(&tty, baudToSpeed(baud_));
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }

        fd_ = fd;
        connected_.store(true, std::memory_order_release);
        resetAllRings();
        logI("SerialMux connected on %s at %u baud", port_.c_str(), baud_);
        return true;
    }

    void closeFd() {
        if (fd_ >= 0) {
            flock(fd_, LOCK_UN);
            ::close(fd_);
            fd_ = -1;
        }
        connected_.store(false, std::memory_order_release);
    }

    void resetAllRings() {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        for (auto &ch : channels_) ch->ring.reset();
    }

    static speed_t baudToSpeed(uint32_t b) {
        switch (b) {
            case 115200:  return B115200;  case 230400:  return B230400;
            case 460800:  return B460800;  case 921600:  return B921600;
            case 1000000: return B1000000; case 2000000: return B2000000;
            default:      return B921600;
        }
    }

    // ── Hilo lector con reconexión automática ─────────────────────────────────
    void readerLoop() {
        using namespace std::chrono_literals;
        auto backoff = 500ms;
        constexpr auto backoff_max = 5000ms;

        while (running_.load()) {
            if (!connected()) {
                logW("SerialMux: attempting reconnect in %lldms",
                     (long long)backoff.count());
                std::this_thread::sleep_for(backoff);
                if (openSerial()) {
                    backoff = 500ms;
                    logI("SerialMux: reconnected");
                } else {
                    backoff = std::min(backoff * 2, backoff_max);
                }
                continue;
            }

            if (!readOneFrame()) {
                // Error de I/O — cerrar y dejar que el loop reintente
                logE("SerialMux: I/O error — closing fd");
                closeFd();
            }
        }
    }

    // ── Parseo de un byte — retorna false en error de I/O ────────────────────
    bool readOneFrame() {
        enum class St : uint8_t {
            READ_ID, WAIT_SOF, READ_INST, READ_LEN, READ_PAYLOAD, READ_CRC
        };

        // Estado persistente entre llamadas — estático al hilo (solo hay uno)
        static St      state{St::READ_ID};
        static uint8_t id_buf[CAN_ID_BYTES]{};
        static uint8_t id_pos{0};
        static uint8_t frame_buf[MAX_FRAME_BYTES]{};
        static uint8_t frame_len{0};
        static uint8_t payload_len{0};
        static uint8_t payload_pos{0};

        auto reset = [&]() {
            state = St::READ_ID; id_pos = 0;
            frame_len = payload_len = payload_pos = 0;
        };

        struct pollfd pfd{fd_, POLLIN, 0};
        int ret = poll(&pfd, 1, 10);
        if (ret < 0) {
            if (errno == EINTR) return true;  // señal — no es error fatal
            return false;
        }
        if (ret == 0) return true;  // timeout — nada que leer

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return false;

        uint8_t b;
        ssize_t n = ::read(fd_, &b, 1);
        if (n < 0) { return errno == EAGAIN; }  // EAGAIN no es error
        if (n == 0) return false;

        switch (state) {
        case St::READ_ID:
            id_buf[id_pos++] = b;
            if (id_pos >= CAN_ID_BYTES) state = St::WAIT_SOF;
            break;
        case St::WAIT_SOF:
            if (b != 0xAA) { reset(); break; }
            frame_len = 0;
            frame_buf[frame_len++] = b;
            state = St::READ_INST;
            break;
        case St::READ_INST:
            frame_buf[frame_len++] = b;
            state = St::READ_LEN;
            break;
        case St::READ_LEN:
            payload_len = b;
            frame_buf[frame_len++] = b;
            payload_pos = 0;
            state = (payload_len == 0) ? St::READ_CRC : St::READ_PAYLOAD;
            break;
        case St::READ_PAYLOAD:
            frame_buf[frame_len++] = b;
            if (++payload_pos >= payload_len) state = St::READ_CRC;
            break;
        case St::READ_CRC:
            frame_buf[frame_len++] = b;
            depositFrame(id_buf, frame_buf, frame_len);
            reset();
            break;
        }
        return true;
    }

    void depositFrame(const uint8_t id_raw[4],
                      const uint8_t *frame, uint8_t len) {
        uint32_t rx_id = (uint32_t)id_raw[0] | ((uint32_t)id_raw[1]<<8)
                       | ((uint32_t)id_raw[2]<<16) | ((uint32_t)id_raw[3]<<24);
        std::lock_guard<std::mutex> lk(channels_mutex_);
        for (auto &ch : channels_) {
            if (ch->rx_id != rx_id) continue;
            for (uint8_t i = 0; i < len; ++i)
                if (!ch->ring.push(frame[i]))
                    logW("SerialMux: ring full rx_id=0x%08X — byte dropped", rx_id);
            return;
        }
        logW("SerialMux: no channel for rx_id=0x%08X — frame dropped", rx_id);
    }

    // ── Logging ───────────────────────────────────────────────────────────────
    template<typename... A>
    void logI(const char *f, A&&... a) const { log(log_info_,  f, std::forward<A>(a)...); }
    template<typename... A>
    void logW(const char *f, A&&... a) const { log(log_warn_,  f, std::forward<A>(a)...); }
    template<typename... A>
    void logE(const char *f, A&&... a) const { log(log_error_, f, std::forward<A>(a)...); }
    template<typename... A>
    static void log(const LogFn &fn, const char *fmt, A&&... a) {
        if (!fn) return;
        char buf[256]; std::snprintf(buf, sizeof(buf), fmt, std::forward<A>(a)...);
        fn(buf);
    }

    // ── Miembros ──────────────────────────────────────────────────────────────
    std::string           port_;
    uint32_t              baud_{921600};
    int                   fd_{-1};
    std::atomic<bool>     connected_{false};
    std::mutex            tx_mutex_;
    std::mutex            channels_mutex_;
    std::thread           reader_thread_;
    std::atomic<bool>     running_{false};
    std::vector<std::unique_ptr<Channel>> channels_;
    LogFn log_info_{}, log_warn_{}, log_error_{};
};