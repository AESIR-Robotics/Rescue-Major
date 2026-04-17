#pragma once
// =============================================================================
// bridge_transport.hpp
//
// Reemplazo de can_transport.hpp. Hereda de TransportInterface y expone 
// la misma API al Protocol_Handler. En lugar de comunicarse con Sockets CAN, 
// delega la IO física al SerialIfaceManager, reensamblando en espacio de usuario.
// =============================================================================

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <linux/can.h>

#include "transport/transport.hpp"
#include "utils/logger.hpp"
#include "serial_iface_manager.hpp"
#include "utils/diagnostics.hpp"
#include "utils/crc.hpp"

using micros     = std::chrono::microseconds;
using stdclock   = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;


class Bridge_Transport : protected TransportInterface {
public:
    enum class Transport_Error {
        NONE, CLOSED, OPEN_FAILED, INVALID_CONFIG, SEG_ERROR
    };

    static constexpr size_t INTERNAL_BUF_SIZE = 255;

    explicit Bridge_Transport(DiagnosticRegistry *reg = nullptr,
                              uint8_t  my_addr   = 0x00,
                              uint8_t  peer_addr = 0x00,
                              uint16_t channel   = 0,
                              uint8_t  priority  = 6)
        : my_addr_{my_addr}, peer_addr_{peer_addr}, channel_{channel}, priority_{priority}
    {
        statusReport.with([this](Status &d){
            d.hardware_id = "Bridge_" + std::to_string(peer_addr_);
            d.level = Status::OK;
            d.message = "The status of Bridge interface " + std::to_string(peer_addr_);
            d.name = "Bridge_Transport_" + std::to_string(my_addr_);
            d.values.emplace(std::make_pair("connected", "false"));
        });
        if(reg) reg->register_source(&statusReport);
    }

    virtual ~Bridge_Transport() noexcept { disconnect(); }

    void setIfaceManager(std::shared_ptr<SerialIfaceManager> mgr) {
        iface_mgr_ = std::move(mgr);
    }

    bool init(DiagnosticRegistry *reg, uint8_t my_addr, uint8_t peer_addr,
              uint16_t channel = 0, uint8_t priority = 6) {
        if(reg) reg->register_source(&statusReport);
        my_addr_ = my_addr; peer_addr_ = peer_addr; 
        channel_ = channel; priority_  = priority;
        log.setName("Bridge_" + std::to_string(peer_addr_));
        return connect();
    }

    std::string getName() override { return "Bridge_" + std::to_string(peer_addr_); }
    bool connected() const override { return is_connected_; }
    bool reconnect() override { return connect(); }

protected:
    size_t writeData(const uint8_t *data, size_t length, deadline_t deadline) override {
        if (!is_connected_ || !data || length == 0) return 0;

        uint8_t frame_buf[CAN_MAX_DLEN];
        if (length <= seg::FRAME_DATA_LEN) {
            frame_buf[0] = seg::make_header(seg::TYPE_SINGLE, tx_seq_);
            tx_seq_ = (tx_seq_ + 1) & 0x3F;
            std::memcpy(frame_buf + 1, data, length);
            return sendFrame(frame_buf, static_cast<uint8_t>(1 + length), deadline) ? length : 0;
        }

        size_t offset = 0; bool first = true;
        while (offset < length) {
            size_t chunk = std::min<size_t>(seg::FRAME_DATA_LEN, length - offset);
            bool last = (offset + chunk >= length);
            uint8_t type = first ? seg::TYPE_START : last ? seg::TYPE_END : seg::TYPE_CONT;

            frame_buf[0] = seg::make_header(type, tx_seq_);
            tx_seq_ = (tx_seq_ + 1) & 0x3F;
            std::memcpy(frame_buf + 1, data + offset, chunk);

            if (!sendFrame(frame_buf, static_cast<uint8_t>(1 + chunk), deadline))
                return 0;

            offset += chunk; first = false;
        }
        return length;
    }

    size_t readData(uint8_t *buffer, size_t length, deadline_t deadline) override {
        if (!is_connected_ || !buffer || length == 0) return 0;
        size_t total = 0;

        while (total < length) {
            if (read_pos_ >= rx_pos_) {
                read_pos_ = 0; rx_pos_ = 0;
                bool complete = false;
                while (!complete) {
                    complete = assemble(deadline);
                    if (!complete && !rx_in_progress_) return total;
                }
            }

            size_t available = rx_pos_ - read_pos_;
            size_t to_copy = std::min(available, length - total);
            std::memcpy(buffer + total, rx_buf_ + read_pos_, to_copy);
            read_pos_ += to_copy;
            total += to_copy;
        }
        return total;
    }

    void disconnectImpl() override {
        if (is_connected_) {
            iface_mgr_->unregisterChannel(rx_id_);
            iface_mgr_->release();
            is_connected_ = false;
            rx_in_progress_ = false;
            statusReport.with([](Status &d){ d.values["connected"] = "false"; });
        }
    }

private:
    bool connect() override {
        disconnectImpl();
        if (!iface_mgr_) return false;

        tx_id_ = can_make_id(my_addr_, peer_addr_, channel_, priority_);
        rx_id_ = can_make_id(peer_addr_, my_addr_, channel_, priority_);

        if (!iface_mgr_->acquire()) return false;

        rx_ring_.reset();
        iface_mgr_->registerChannel(rx_id_, &rx_ring_);

        tx_seq_ = 0; rx_pos_ = 0; rx_in_progress_ = false;
        is_connected_ = true;
        
        statusReport.with([](Status &d){ d.values["connected"] = "true"; });
        return true;
    }

    bool sendFrame(const uint8_t *frame_data, uint8_t frame_len, deadline_t deadline) {
        while (true) {
            if (stdclock::now() >= deadline) return false;
            // Escribir hacia el manager
            if (iface_mgr_->writeFrame(tx_id_, frame_len, frame_data)) return true;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    uint8_t recvFrame(uint8_t *frame_data, deadline_t deadline) {
        struct can_frame frame;
        while (true) {
            // Non-blocking poll del MuxRing
            if (rx_ring_.pop(frame)) {
                uint8_t dlc = (frame.can_dlc > CAN_MAX_DLEN) ? CAN_MAX_DLEN : frame.can_dlc;
                std::memcpy(frame_data, frame.data, dlc);
                return dlc;
            }
            if (stdclock::now() >= deadline) return 0;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    bool assemble(deadline_t deadline) {
        uint8_t frame_buf[CAN_MAX_DLEN];
        uint8_t dlc = recvFrame(frame_buf, deadline);
        if (dlc < 1) return false;

        uint8_t  header  = frame_buf[0];
        uint8_t  type    = seg::frame_type(header);
        uint8_t  seq     = seg::frame_seq(header);
        uint8_t  datalen = dlc - 1;
        uint8_t *fdata   = frame_buf + 1;

        switch (type) {
        case seg::TYPE_SINGLE:
            rx_in_progress_  = false;
            rx_expected_seq_ = (seq + 1) & 0x3F;
            std::memcpy(rx_buf_, fdata, datalen);
            rx_pos_ = datalen; read_pos_ = 0;
            return true;
        case seg::TYPE_START:
            rx_in_progress_  = true; rx_pos_ = 0; read_pos_ = 0;
            rx_expected_seq_ = (seq + 1) & 0x3F;
            std::memcpy(rx_buf_, fdata, datalen);
            rx_pos_ = datalen;
            return false;
        case seg::TYPE_CONT:
            if (!rx_in_progress_ || seq != rx_expected_seq_) { rx_in_progress_ = false; return false; }
            rx_expected_seq_ = (seq + 1) & 0x3F;
            std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
            rx_pos_ += datalen;
            return false;
        case seg::TYPE_END:
            if (!rx_in_progress_ || seq != rx_expected_seq_) { rx_in_progress_ = false; return false; }
            std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
            rx_pos_ += datalen; rx_in_progress_ = false; read_pos_ = 0;
            return true;
        }
        return false;
    }

    uint8_t  my_addr_;
    uint8_t  peer_addr_;
    uint16_t channel_;
    uint8_t  priority_;

    uint32_t tx_id_{ 0 };
    uint32_t rx_id_{ 0 };
    bool     is_connected_{ false };

    std::shared_ptr<SerialIfaceManager> iface_mgr_;
    MuxRing<struct can_frame, 256> rx_ring_; 

    uint8_t  tx_seq_{ 0 };
    uint8_t  rx_buf_[INTERNAL_BUF_SIZE]{};
    size_t   rx_pos_{ 0 };
    size_t   read_pos_{ 0 };
    uint8_t  rx_expected_seq_{ 0 };
    bool     rx_in_progress_{ false };

    Tracked<Status> statusReport;
};