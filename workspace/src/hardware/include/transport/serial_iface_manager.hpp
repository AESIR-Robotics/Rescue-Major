#pragma once
// =============================================================================
// serial_iface_manager.hpp
//
// Gestor compartido de la interfaz Serial física actuando como bus CAN.
// Múltiples instancias de Bridge_Transport se registran a este manager.
// Utiliza una ventana deslizante de 15 bytes para evitar desincronización.
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <linux/can.h>

#include "utils/logger.hpp"
#include "utils/diagnostics.hpp"

// =============================================================================
// MuxRing genérico lock-free para paso de mensajes (SPSC)
// =============================================================================
template <typename T, size_t SIZE>
class MuxRing {
public:
    bool push(const T& item) {
        size_t next = (head_.load(std::memory_order_relaxed) + 1) % SIZE;
        if (next == tail_.load(std::memory_order_acquire)) return false;
        buf_[head_.load(std::memory_order_relaxed)] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T& item) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        item = buf_[t];
        tail_.store((t + 1) % SIZE, std::memory_order_release);
        return true;
    }
    void reset() {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }
private:
    std::atomic<size_t> head_{0}, tail_{0};
    T buf_[SIZE]{};
};

// =============================================================================
// SerialIfaceManager
// =============================================================================
class SerialIfaceManager {
public:
    static constexpr uint8_t BRIDGE_SOF = 0xAA;
    static constexpr size_t  FRAME_SIZE = 15;

    struct Channel {
        uint32_t rx_id { 0 };
        MuxRing<struct can_frame, 256>* ring { nullptr };
    };

    explicit SerialIfaceManager(DiagnosticRegistry *reg = nullptr, 
                                std::string port = "", 
                                uint32_t baudrate = 115200)
        : port_{std::move(port)}, baudrate_{baudrate} 
    {
        statusReport.with([this](Status &d){
            d.hardware_id = port_;
            d.level = Status::OK;
            d.message = "The status of the Serial-CAN bridge interface";
            d.name = "Serial_Interface";
            d.values.emplace(std::make_pair("connected", "false"));
            d.values.emplace(std::make_pair("instances", "0"));
        });
        if(reg) reg->register_source(&statusReport);
    }

    ~SerialIfaceManager() { stop(); }

    void setLogger(Logger &in_log) { log = in_log; }

    bool acquire() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (ref_count_ == 0) {
            if (!openSerial()) return false;
            startReader();
        }
        ++ref_count_;
        statusReport.with([this](Status &d){
            d.values["connected"] = "true";
            d.values["instances"] = std::to_string(ref_count_);
        });
        return true;
    }

    void release() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (--ref_count_ == 0) {
            stop();
        }
        statusReport.with([this](Status &d){
            d.values["instances"] = std::to_string(ref_count_);
        });
    }

    // Registra el anillo de recepción para un CAN_ID específico
    void registerChannel(uint32_t rx_id, MuxRing<struct can_frame, 256>* ring) {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        for (auto &ch : channels_) {
            if (ch.rx_id == rx_id) { ch.ring = ring; return; }
        }
        channels_.push_back({rx_id, ring});
    }

    void unregisterChannel(uint32_t rx_id) {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        channels_.erase(std::remove_if(channels_.begin(), channels_.end(),
                        [rx_id](const Channel& c) { return c.rx_id == rx_id; }), 
                        channels_.end());
    }

    // Transmisión desde Bridge_Transport
    bool writeFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data) {
        if (!connected_.load()) return false;
        
        uint8_t buf[FRAME_SIZE];
        buf[0] = BRIDGE_SOF;
        buf[1] = tx_seq_++; 
        buf[2] = can_id & 0xFF;
        buf[3] = (can_id >> 8) & 0xFF;
        buf[4] = (can_id >> 16) & 0xFF;
        buf[5] = (can_id >> 24) & 0xFF;
        buf[6] = dlc;
        
        for (int i = 0; i < 8; i++) {
            buf[7 + i] = (i < dlc) ? data[i] : 0xBB;
        }

        std::lock_guard<std::mutex> lk(tx_mutex_);
        ssize_t sent = ::write(fd_, buf, FRAME_SIZE);
        return (sent == FRAME_SIZE);
    }

private:
    bool openSerial() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            log.logError("SerialManager open(%s): %s", port_.c_str(), strerror(errno));
            return false;
        }
        
        struct termios tty{};
        tcgetattr(fd_, &tty);
        cfmakeraw(&tty);
        speed_t speed = baudToSpeed(baudrate_);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0; 
        tty.c_cc[VTIME] = 0;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { 
            ::close(fd_); fd_ = -1; return false; 
        }
        
        connected_.store(true);
        log.logInfo("SerialManager connected on %s at %u baud", port_.c_str(), baudrate_);
        return true;
    }

    void stop() {
        running_.store(false);
        if (reader_thread_.joinable()) reader_thread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        connected_.store(false);
        statusReport.with([](Status &d){ d.values["connected"] = "false"; });
    }

    void startReader() {
        running_.store(true);
        reader_thread_ = std::thread(&SerialIfaceManager::readerLoop, this);
    }

    void readerLoop() {
        while (running_.load()) {
            if (!connected_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            struct pollfd pfd{ fd_, POLLIN, 0 };
            int ret = poll(&pfd, 1, 100);
            if (ret <= 0) continue;

            ssize_t n = ::read(fd_, rx_stream_ + rx_pos_, sizeof(rx_stream_) - rx_pos_);
            if (n <= 0) continue;
            rx_pos_ += n;

            // --- Parsing y Sincronización Ventana Deslizante ---
            while (rx_pos_ >= FRAME_SIZE) {
                if (rx_stream_[0] != BRIDGE_SOF) {
                    // Desync detectado, deslizar 1 byte y buscar el ancla
                    std::memmove(rx_stream_, rx_stream_ + 1, --rx_pos_);
                    continue;
                }

                // Extraer frame
                struct can_frame frame{};
                uint32_t id = rx_stream_[2] | (rx_stream_[3]<<8) | (rx_stream_[4]<<16) | (rx_stream_[5]<<24);
                frame.can_id = id;
                frame.can_dlc = (rx_stream_[6] > 8) ? 8 : rx_stream_[6];
                std::memcpy(frame.data, &rx_stream_[7], frame.can_dlc);

                routeFrame(frame);

                // Consumir trama completa y deslizar
                rx_pos_ -= FRAME_SIZE;
                if (rx_pos_ > 0) {
                    std::memmove(rx_stream_, rx_stream_ + FRAME_SIZE, rx_pos_);
                }
            }
        }
    }

    void routeFrame(const struct can_frame& frame) {
        std::lock_guard<std::mutex> lk(channels_mutex_);
        for (auto &ch : channels_) {
            if (ch.rx_id == frame.can_id && ch.ring) {
                if (!ch.ring->push(frame)) {
                    log.logWarn("SerialManager: Ring full for ID 0x%08X", frame.can_id);
                }
                break; // Asumimos 1:1 ruteo por ID
            }
        }
    }

    static speed_t baudToSpeed(uint32_t b) {
        switch (b) {
            case 115200:  return B115200;  case 921600:  return B921600;
            case 1000000: return B1000000; case 2000000: return B2000000;
            default:      return B921600;
        }
    }

    std::string port_;
    uint32_t baudrate_;
    int fd_{ -1 };
    int ref_count_{ 0 };
    uint8_t tx_seq_{ 0 };
    
    std::atomic<bool> connected_{ false };
    std::atomic<bool> running_{ false };
    
    std::mutex mutex_;
    std::mutex tx_mutex_;
    std::mutex channels_mutex_;
    
    std::thread reader_thread_;
    std::vector<Channel> channels_;

    // Buffer del stream serial
    uint8_t rx_stream_[FRAME_SIZE * 4]{}; 
    size_t rx_pos_{ 0 };

    Tracked<Status> statusReport;
    Logger log;
};