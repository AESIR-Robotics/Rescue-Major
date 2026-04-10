#pragma once
// =============================================================================
// bridge_transport.hpp
//
// Currently unused, statusReport is unoperative
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

#include "utils/diagnostics.hpp"
#include "utils/logger.hpp"

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

    explicit Bridge_Transport(DiagnosticRegistry *reg){
        statusReport.with([](Status &d){
            d.hardware_id = "Bridge_Serial_Status_Unused";
            d.level = Status::OK;
            d.message = "";
            d.name = "Bridge_Serial_Status_Unused";
            d.values.emplace(
            std::make_pair("attmps", "0")
            );
        });
        if(reg){
            reg->register_source(&statusReport);
        }
    }

    // ── init — toma shared_ptr para garantizar lifetime del mux ──────────────
    bool init(DiagnosticRegistry *reg, std::shared_ptr<SerialMux> mux,
              uint8_t  my_addr,
              uint8_t  peer_addr,
              uint16_t channel  = 0,
              uint8_t  priority = 6) {
        if(reg){
            reg->register_source(&statusReport);
        }
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

    // Add better feedback on to when they can send
    // And avoid accidentally not giving priority to other channels 
    // the same mux. 
    // (If a lot of devices try to send and the first few block the mux
    // then they will always block it before other ones get a chance of sending
    // essentially blocking them from ever sending through the mux)
    bool canSend() const {
        return true;
    }

    // ── hasData — consultado por Protocol_Handler antes del sync scan ─────────
    bool hasData() const {
        return channel_ && !channel_->ring.empty();
    }

    void setLogger(Logger &in_log) {
        log = in_log;
    }

    Transport_Error transport_error_{ Transport_Error::CLOSED };

protected:
    // ── writeData — manda en unidades fijas de UNIT_BYTES (12) ───────────────
    // El frame del protocolo puede ser mayor que MAX_CAN_DATA (8 bytes).
    // Se corta en chunks de 8 bytes, cada uno enviado como una unidad
    // independiente con el mismo tx_id. El Arduino los manda como frames
    // CAN separados con el mismo ID. El Protocol_Handler en el MCU destino
    // reconstruye el frame desde el stream — la segmentación es transparente.
    size_t writeData(const uint8_t *data, size_t length,
                     deadline_t /*deadline*/) {
        if (!connected() || !data || length == 0) return 0;

        constexpr size_t ID_B   = SerialMux::CAN_ID_BYTES;  // 4
        constexpr size_t DATA_B = SerialMux::MAX_CAN_DATA;  // 8
        constexpr size_t UNIT   = SerialMux::UNIT_BYTES;    // 12

        uint8_t unit[UNIT + 1];
        unit[0] = 0xAA;
        SerialMux::toLE(tx_id_, unit + 1);  // ID siempre igual

        size_t offset = 0;
        while (offset < length) {
            size_t chunk = std::min(DATA_B, length - offset);
            std::memcpy(unit + ID_B + 1, data + offset, chunk);
            // Paddear con 0xBB si el chunk es menor que MAX_CAN_DATA
            if (chunk < DATA_B)
                std::memset(unit + ID_B + 1 + chunk, 0xBB, DATA_B - chunk);

            size_t sent = mux_->write(unit, UNIT + 1);
            if (sent != UNIT + 1) return 0;  // error de TX — abortar
            offset += chunk;
        }
        return length;
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

    Tracked<Status> statusReport;

private:
    std::shared_ptr<SerialMux> mux_;
    SerialMux::Channel        *channel_{ nullptr };
    uint32_t                   tx_id_  { 0 };
    uint32_t                   rx_id_  { 0 };

    Logger log{};

};

// =============================================================================
// Alias
// =============================================================================
#include "protocol_handler.hpp"

template<typename ReadID  = Cmd::ESP32::Read,
         typename WriteID = Cmd::ESP32::Write>
using Protocol_Handler_Bridge = Protocol_Handler<Bridge_Transport, ReadID, WriteID>;