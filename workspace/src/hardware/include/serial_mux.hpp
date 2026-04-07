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
    static constexpr size_t MAX_CAN_DATA    = 8;   ///< Max data bytes per CAN 2.0B frame
    static constexpr size_t UNIT_BYTES      = CAN_ID_BYTES + MAX_CAN_DATA; ///< 12 — fixed serial unit
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

        using clock = std::chrono::steady_clock;

        auto now = clock::now();

        auto tx_duration = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(10.0 * length / baud_)
        );

        // si vamos adelantados, esperamos
        if (now < next_tx_time_) {
            std::this_thread::sleep_until(next_tx_time_);
            now = next_tx_time_;
        }

        // escribe
        ssize_t sent = ::write(fd_, data, length);
        if (sent < 0) {
            logE("SerialMux write() failed: %s", strerror(errno));
            return 0;
        }

        // actualiza siguiente tiempo permitido
        next_tx_time_ = now + tx_duration;

        return static_cast<size_t>(sent);
    }

    // Expuesto para que Bridge_Transport arme el frame TX
    static void toLE(uint32_t id, uint8_t b[4]) {
        b[0]=id&0xFF; b[1]=(id>>8)&0xFF;
        b[2]=(id>>16)&0xFF; b[3]=(id>>24)&0xFF;
    }

    static uint32_t fromLE(uint8_t b[4]) {
        return b[0] | b[1]<<8 | b[2]<<16 | b[3]<<24;
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
    //
    // Lee en unidades fijas de UNIT_BYTES (12):
    //   [ID 4B LE][data 8B]
    //
    // Sincronización: el primer byte del ID debe tener los bits de priority
    // (bits 7-5) en el rango válido de tu esquema. Si no, se avanza byte a
    // byte hasta encontrar un ID válido. Esto permite re-sincronizarse si
    // el SerialMux arranca en medio de una unidad.
    //
    // El Arduino paddea frames CAN cortos con 0xBB — el SerialMux los deposita
    // en el ring igual. El Protocol_Handler los filtra como bytes no-0xAA
    // durante el sync scan, igual que siempre.
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

            if (!readUnits()) {
                logE("SerialMux: I/O error — closing fd");
                closeFd();
            }
        }
    }

    // ── Lector de unidades fijas — retorna false en error de I/O ─────────────
    //
    // Estado de sincronización: unit_buf_ acumula bytes hasta tener UNIT_BYTES.
    // Si el ID extraído no es válido, descarta 1 byte y reintenta (slide window).
    bool readUnits() {
        struct pollfd pfd{fd_, POLLIN, 0};
        int ret = poll(&pfd, 1, 10);
        if (ret < 0)  { return errno == EINTR; }
        if (ret == 0) { return true; }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return false;

        if (!synced) {
            uint8_t byte;
            ssize_t n = ::read(fd_, &byte, 1);
            if (n < 0)  { return errno == EAGAIN; }
            if (n == 0) { return false; }

            if (byte == 0xAA) {
                synced = true;
                unit_pos_ = 0; // empezar limpio después del sync
            }else{
                return true; // ignorar todo hasta encontrar 0xAA
            }
        }
        
        size_t space = UNIT_BYTES - unit_pos_;
        ssize_t n = ::read(fd_, unit_buf_ + unit_pos_, space);
        unit_pos_ += static_cast<size_t>(n);

        if (n < 0)  { return errno == EAGAIN; }
        if (n == 0) { return false; }

        while (unit_pos_ >= UNIT_BYTES) {
            uint32_t rx_id = fromLE(unit_buf_);
            synced = false;

            if (isValidCanId(rx_id)) {
                depositData(unit_buf_, rx_id);

                // consumir unidad
                unit_pos_ -= UNIT_BYTES;
                if (unit_pos_ > 0)
                    std::memmove(unit_buf_, unit_buf_ + UNIT_BYTES, unit_pos_);

                
            } else {
                logW("SerialMux: bad frame — resyncing");
                unit_pos_ = 0;
            }
        }

        return true;
    }

    // ── Validar ID según el esquema de bits ───────────────────────────────────
    // [28..26] priority (3 bits, valores 0-7)
    // [25..18] sender   (8 bits)
    // [17..10] receiver (8 bits)
    // [9..0]   channel  (10 bits)
    // Un ID es válido si está dentro del rango de 29 bits (bit 29+ = 0).
    // Adicionalmente rechazamos 0x00000000 y 0x1FFFFFFF como IDs de relleno.
    static bool isValidCanId(uint32_t id) {
        if (id == 0x00000000UL) return false;  // ID nulo — no válido
        if (id > 0x1FFFFFFFUL) return false;   // supera 29 bits
        return true;
    }

    // ── Depositar los 8 bytes de data en el canal correcto ───────────────────
    // unit_buf apunta al inicio de la unidad de 12 bytes.
    // Los primeros 4 bytes son el ID (ya extraído como rx_id).
    // Los bytes CAN_ID_BYTES..UNIT_BYTES-1 son los datos a depositar.
    // Los 0xBB de padding son bytes válidos desde la perspectiva del ring —
    // el Protocol_Handler los descarta durante el sync scan como bytes != 0xAA.
    void depositData(const uint8_t *unit_buf, uint32_t rx_id) {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        for (auto &ch : channels_) {
            if (ch->rx_id != rx_id) continue;
            for (size_t i = CAN_ID_BYTES; i < UNIT_BYTES; ++i)
                if (!ch->ring.push(unit_buf[i]))
                    logW("SerialMux: ring full rx_id=0x%08X — byte dropped", rx_id);
            logI("Got info, rx_id=0x%08X", rx_id);
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
    std::chrono::steady_clock::time_point next_tx_time_ = std::chrono::steady_clock::now();
    int                   fd_{-1};
    std::atomic<bool>     connected_{false};
    std::mutex            tx_mutex_;
    std::mutex            channels_mutex_;
    std::thread           reader_thread_;
    std::atomic<bool>     running_{false};
    std::vector<std::unique_ptr<Channel>> channels_;
    LogFn log_info_{}, log_warn_{}, log_error_{};

    // Buffer de sincronización del lector de unidades fijas
    uint8_t unit_buf_[UNIT_BYTES * 2]{};  // x2 para el slide window
    size_t  unit_pos_{ 0 };
    bool synced {false};
};