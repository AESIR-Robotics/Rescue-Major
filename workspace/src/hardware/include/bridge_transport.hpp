#pragma once
// =============================================================================
// bridge_transport.hpp
//
// Transport que conecta Protocol_Handler con SerialMux.
// Cada instancia registra su canal RX y comparte el fd serial via shared_ptr.
//
// Error recovery:
//   reconnect() delega al mux — si cualquier instancia lo llama y el mux
//   se reconecta, todos los brazos se benefician automáticamente.
//   En hardware_driver_node, errorRecoveryCANArm(i) llama
//   stepper_arms[i].reconnect() — si retorna true activa can_needs_resync_.
// =============================================================================

#include <memory>
#include "serial_mux.hpp"
#include "can_transport.hpp"   // para can_make_id

using micros     = std::chrono::microseconds;
using stdclock   = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;

class Bridge_Transport {
public:
    enum class Transport_Error {
        NONE,
        CLOSED,
        INVALID_CONFIG,
    };

    static constexpr size_t INTERNAL_BUF_SIZE = 64;

    Bridge_Transport() = default;

    // ── init — toma shared_ptr para garantizar lifetime del mux ──────────────
    bool init(std::shared_ptr<SerialMux> mux,
              uint8_t  my_addr,
              uint8_t  peer_addr,
              uint16_t channel  = 0,
              uint8_t  priority = 6) {
        mux_     = std::move(mux);
        tx_id_   = can_make_id(my_addr, peer_addr, channel, priority);
        rx_id_   = can_make_id(peer_addr, my_addr, channel, priority);
        channel_ = mux_->registerChannel(rx_id_);

        if (!channel_) {
            transport_error_ = Transport_Error::INVALID_CONFIG;
            return false;
        }
        transport_error_ = Transport_Error::NONE;
        return true;
    }

    // ── Interface requerida por Protocol_Handler ──────────────────────────────
    bool connected() const {
        return mux_ && mux_->connected() && channel_;
    }

    std::string getDevice()       const { return mux_ ? "serial_bridge" : ""; }
    int  getSlaveAddress()        const { return static_cast<int>(rx_id_); }
    Transport_Error getTransportError() const { return transport_error_; }

    // ── reconnect — delega al mux, beneficia a TODAS las instancias ───────────
    bool reconnect() {
        if (!mux_) { transport_error_ = Transport_Error::INVALID_CONFIG; return false; }
        bool ok = mux_->reconnect();
        transport_error_ = ok ? Transport_Error::NONE : Transport_Error::CLOSED;
        return ok;
    }

    // ── hasData — consultado por Protocol_Handler antes del sync scan ─────────
    bool hasData() const {
        return channel_ && !channel_->ring.empty();
    }

    void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
        log_info_  = std::move(info);
        log_warn_  = std::move(warn);
        log_error_ = std::move(error);
    }

    Transport_Error transport_error_{ Transport_Error::CLOSED };

protected:
    // ── writeData — prepend tx_id y escribe al mux ────────────────────────────
    size_t writeData(const uint8_t *data, size_t length,
                     deadline_t /*deadline*/) {
        if (!connected() || !data || length == 0) return 0;

        const size_t total = SerialMux::CAN_ID_BYTES + length;
        uint8_t buf[SerialMux::CAN_ID_BYTES + SerialMux::MAX_FRAME_BYTES];
        SerialMux::toLE(tx_id_, buf);
        std::memcpy(buf + SerialMux::CAN_ID_BYTES, data, length);

        size_t sent = mux_->write(buf, total);
        return (sent == total) ? length : 0;
    }

    // ── readData — drena el ring local del canal ──────────────────────────────
    // No toca el fd. El hilo del mux deposita los bytes aquí.
    // Si el ring está vacío espera 50µs y reintenta hasta el deadline.
    size_t readData(uint8_t *buffer, size_t length, deadline_t deadline) {
        if (!connected() || !buffer || length == 0) return 0;

        size_t total = 0;
        while (total < length) {
            if (stdclock::now() >= deadline) break;
            uint8_t b;
            if (channel_->ring.pop(b)) {
                buffer[total++] = b;
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
        return total;
    }

private:
    std::shared_ptr<SerialMux> mux_;
    SerialMux::Channel        *channel_{ nullptr };
    uint32_t                   tx_id_  { 0 };
    uint32_t                   rx_id_  { 0 };

    LogFn log_info_{}, log_warn_{}, log_error_{};

    template<typename... A>
    void logW(const char *f, A&&... a) const {
        if (!log_warn_) return;
        char buf[256]; std::snprintf(buf, sizeof(buf), f, std::forward<A>(a)...);
        log_warn_(buf);
    }
};

// =============================================================================
// Alias
// =============================================================================
#include "protocol_handler.hpp"

template<typename ReadID  = Cmd::ESP32::Read,
         typename WriteID = Cmd::ESP32::Write>
using Protocol_Handler_Bridge = Protocol_Handler<Bridge_Transport, ReadID, WriteID>;