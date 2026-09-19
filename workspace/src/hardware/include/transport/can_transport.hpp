#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <poll.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "transport/transport.hpp"
#include "utils/logger.hpp"
#include "can_iface_manager.hpp"
#include "utils/diagnostics.hpp"
#include "utils/crc.hpp"

using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;
using LogFn      = std::function<void(const std::string &)>;


// =============================================================================
// CAN_Transport
//
// Owns a SocketCAN file descriptor and provides CAN 2.0B segmented I/O.
// Public/protected interface is identical to the original so that
// Protocol_Handler<CAN_Transport> compiles without changes.
// =============================================================================

class CAN_Transport : protected TransportInterface{
public:
    enum class Transport_Error {
        NONE,
        CLOSED,
        OPEN_FAILED,
        INVALID_CONFIG,
        IOCTL_FAILED,
        FD_INVALID,
        SEG_ERROR,       ///< Sequence/reassembly mismatch
    };

    // Reassembly scratch buffer.  Increase if messages exceed this size.
    static constexpr size_t INTERNAL_BUF_SIZE = 255;

    explicit CAN_Transport(DiagnosticRegistry *reg = nullptr, const std::string &interface = "",
                           uint8_t  my_addr   = 0x00,
                           uint8_t  peer_addr = 0x00,
                           uint16_t channel   = 0,
                           uint8_t  priority  = 6);
    virtual ~CAN_Transport() noexcept;

    CAN_Transport(const CAN_Transport &)            = delete;
    CAN_Transport &operator=(const CAN_Transport &) = delete;

    bool init(DiagnosticRegistry *reg, const std::string &interface,
              uint8_t  my_addr,
              uint8_t  peer_addr,
              uint16_t channel  = 0,
              uint8_t  priority = 6);
    std::string getName() override;
    bool connect() override;
    bool reconnect() override;

     bool               connected() const override { return sock_fd_ >= 0;    }
    const std::string &getDevice()         const { return interface_;       }
    int                getSlaveAddress()   const { return peer_addr_;       }
    Transport_Error    getTransportError() const { return transport_error_; }

    /// Set the shared interface manager. Call before init() or connect().
    /// If not set, CAN_Transport manages the interface on its own (legacy).
    void setIfaceManager(std::shared_ptr<CANIfaceManager> mgr) {
        iface_mgr_ = std::move(mgr);
    }

protected:
    // ── Called by Protocol_Handler ──────────────────────────────────────────

    /// Segment `data` into CAN 2.0B frames and transmit all of them.
    /// Returns bytes written (0 on failure).
    size_t writeData(const uint8_t *data, size_t length, deadline_t deadline) override;

    bool canSend() const override;
    bool hasData() const override;

    bool assemble(deadline_t deadline);

    /// Receive and reassemble segmented CAN 2.0B frames into `buffer`.
    /// Returns bytes copied.
    size_t readData(uint8_t *buffer, size_t length, deadline_t deadline) override;

    void disconnectImpl() override;

    Transport_Error transport_error_ { Transport_Error::CLOSED };

    Tracked<Status> statusReport; 

private:

    void disconnect_priv();

    bool waitWritableUntil(deadline_t deadline);

    // ── Segmentation helpers ────────────────────────────────────────────────
    bool    sendFrame(const uint8_t *frame_data, uint8_t frame_len,
                      deadline_t deadline);
    uint8_t recvFrame(uint8_t *frame_data, deadline_t deadline);

    void logBufferHex(const Logger& log,
                         const char* prefix,
                         const uint8_t* data,
                         uint8_t len);

    // ── Socket helpers ──────────────────────────────────────────────────────
    bool isValidConfig() const noexcept;
    bool waitFdReady(short events, deadline_t deadline);

    // ── Members ─────────────────────────────────────────────────────────────
    std::string interface_;
    uint8_t     my_addr_   { 0x00 };
    uint8_t     peer_addr_ { 0x00 };
    uint16_t    channel_   { 0    };
    uint8_t     priority_  { 6    };

    uint32_t    tx_id_     { 0 };
    uint32_t    rx_id_     { 0 };
    int         sock_fd_   { -1 };
    std::shared_ptr<CANIfaceManager> iface_mgr_;  ///< shared interface manager

    // TX: 6-bit rolling sequence counter
    uint8_t     tx_seq_    { 0 };

    // RX reassembly state
    uint8_t     rx_buf_[INTERNAL_BUF_SIZE]{};
    size_t      rx_pos_            { 0     };
    size_t      read_pos_          { 0     };
    uint8_t     rx_expected_seq_   { 0     };
    bool        rx_in_progress_    { false };

    u_int32_t  attmps { 0 };
};

// =============================================================================
// Inline definitions
// =============================================================================

inline CAN_Transport::CAN_Transport(DiagnosticRegistry *reg, const std::string &interface,
                                     uint8_t my_addr, uint8_t peer_addr,
                                     uint16_t channel, uint8_t priority)
    : interface_{interface}, my_addr_{my_addr}, peer_addr_{peer_addr},
      channel_{channel}, priority_{priority}
{   
    statusReport.with([this](Status &d){
        // hardware_id is [interface]_[peer_addr]
        // message is "The status of can interface [peer_addr]"
        // name is CAN_Transport_[my_addr]

        d.hardware_id = interface_ + "_" + std::to_string(peer_addr_);
        d.level = Status::OK;
        d.message = "The status of CAN interface " + std::to_string(peer_addr_);
        d.name = "CAN_Transport_" + std::to_string(my_addr_);
        d.values.emplace(std::make_pair("connected", "false"));
        d.values.emplace(std::make_pair("con_attmps", "0"));
        d.values.emplace(std::make_pair("kernel", ""));
    });
    if(reg){
        reg->register_source(&statusReport);
    }
    if (!interface_.empty()) connect();
}

inline CAN_Transport::~CAN_Transport() noexcept { disconnect(); }

inline std::string CAN_Transport::getName(){
    return "CAN_" + std::to_string(peer_addr_);
}

inline bool CAN_Transport::init(DiagnosticRegistry *reg, const std::string &interface,
                                 uint8_t my_addr, uint8_t peer_addr,
                                 uint16_t channel, uint8_t priority) {
    if(reg){
        reg->register_source(&statusReport);
    }
    interface_ = interface;
    my_addr_   = my_addr;
    peer_addr_ = peer_addr;
    channel_   = channel;
    priority_  = priority;
    log.setName("CAN_" + std::to_string(peer_addr_));
    return connect();
}

inline bool CAN_Transport::reconnect() {
    if (interface_.empty()) {
        transport_error_ = Transport_Error::INVALID_CONFIG;
        return false;
    }
    return connect();
}

inline bool CAN_Transport::connect() {
    disconnect_priv();
    transport_error_ = Transport_Error::NONE;
    attmps++;
    statusReport.with([this](Status &d){
        d.values["con_attmps"] = std::to_string(attmps);
    });

    if (!isValidConfig()) {
        transport_error_ = Transport_Error::INVALID_CONFIG;
        log.logError("CAN has invalid config");
        statusReport.with([](Status &d){d.level = Status::ERROR;});
        return false;
    }

    // Bring interface up via shared manager (checks state, avoids redundant ops)
    if (iface_mgr_) {
        if (!iface_mgr_->acquire()) {
            transport_error_ = Transport_Error::OPEN_FAILED;
            log.logError("CAN: interface manager failed to bring up %s",
                         interface_.c_str());
            statusReport.with([](Status &d){d.level = Status::ERROR;});
            return false;
        }
    }

    tx_id_ = can_make_id(my_addr_,   peer_addr_, channel_, priority_);
    rx_id_ = can_make_id(peer_addr_, my_addr_,   channel_, priority_);

    // Classic CAN 2.0B socket — do NOT enable CAN_RAW_FD_FRAMES
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        log.logError("CAN socket() failed: %s", strerror(errno));
        statusReport.with([](Status &d){d.level = Status::ERROR;});
        return false;
    }

    // Hardware filter: only receive frames matching rx_id_
    struct can_filter rfilter{};
    rfilter.can_id   = rx_id_;
    rfilter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
    if (setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
                   &rfilter, sizeof(rfilter)) < 0) {
        transport_error_ = Transport_Error::IOCTL_FAILED;
        log.logError("CAN_RAW_FILTER setsockopt failed: %s", strerror(errno));
        disconnect();
        statusReport.with([](Status &d){d.level = Status::ERROR;});
        return false;
    }

    // Bind to interface
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        log.logError("CAN interface %s not found: %s", interface_.c_str(), strerror(errno));
        disconnect();
        statusReport.with([](Status &d){d.level = Status::ERROR;});
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        log.logError("CAN bind() failed: %s", strerror(errno));
        disconnect();
        statusReport.with([](Status &d){d.level = Status::ERROR;});
        return false;
    }

    // Non-blocking I/O
    int flags = fcntl(sock_fd_, F_GETFL, 0);
    fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    // Reset segmentation state
    tx_seq_         = 0;
    rx_pos_         = 0;
    rx_in_progress_ = false;

    log.logInfo("CAN 2.0B connected on %s  tx=0x%08X  rx=0x%08X",
            interface_.c_str(), tx_id_, rx_id_);
    statusReport.with([](Status &d){
        d.level = Status::OK;
        d.values["connected"] = "true";
        d.values["kernel"] = "";
    });
    return true;
}

inline void CAN_Transport::disconnectImpl() {
    if (sock_fd_ >= 0) {
        statusReport.with([](Status &d){
            d.values["connected"] = "false";
        });
        ::close(sock_fd_);
        sock_fd_        = -1;
        rx_pos_         = 0;
        rx_in_progress_ = false;
        log.logInfo("CAN socket closed on %s", interface_.c_str());
        if (transport_error_ == Transport_Error::NONE)
            transport_error_ = Transport_Error::CLOSED;
        
        if (iface_mgr_) iface_mgr_->release();
    }
    
}

inline void CAN_Transport::disconnect_priv() {
    if (sock_fd_ >= 0) {
        statusReport.with([](Status &d){
            d.values["connected"] = "false";
        });
        ::close(sock_fd_);
        sock_fd_        = -1;
        rx_pos_         = 0;
        rx_in_progress_ = false;
        log.logInfo("CAN socket closed on %s", interface_.c_str());
        if (transport_error_ == Transport_Error::NONE)
            transport_error_ = Transport_Error::CLOSED;
    }
}

inline bool CAN_Transport::isValidConfig() const noexcept {
    return !interface_.empty();
}

inline bool CAN_Transport::waitFdReady(short events, deadline_t deadline) {
    struct pollfd pfd{};
    pfd.fd     = sock_fd_;
    pfd.events = events;

    auto remaining_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - stdclock::now());
    if (remaining_ns.count() <= 0) return false;

    struct timespec ts{};
    ts.tv_sec  = remaining_ns.count() / 1'000'000'000LL;
    ts.tv_nsec = remaining_ns.count() % 1'000'000'000LL;

    int ret = ppoll(&pfd, 1, &ts, nullptr);
    if (ret < 0) {
        if (errno == EBADF) {
            disconnect();
            transport_error_ = Transport_Error::FD_INVALID;
            statusReport.with([](Status &d){
                d.level = Status::ERROR;
                d.values["kernel"] = strerror(errno);
            });
        }
        return false;
    }
    return ret > 0 && (pfd.revents & events);
}


inline bool CAN_Transport::canSend() const {
    if (!connected()) return false;

    struct pollfd pfd{ sock_fd_, POLLOUT, 0 };

    // timeout = 0 → no bloquea
    int ret = poll(&pfd, 1, 0);

    return ret > 0 && (pfd.revents & POLLOUT);
}

inline bool CAN_Transport::hasData() const {
    if (!connected()) return false;
    // If there are already assembled bytes ready to read, return immediately
    if (read_pos_ < rx_pos_) return true;
    // Non-blocking poll — 0 timeout means return instantly
    struct pollfd pfd{ sock_fd_, POLLIN, 0 };
    int ret = poll(&pfd, 1, 0);
    return ret > 0 && (pfd.revents & POLLIN);
}


inline void CAN_Transport::logBufferHex(const Logger& log,
                         const char* prefix,
                         const uint8_t* data,
                         uint8_t len)
{
    if (!data || len == 0) {
        log.logInfo("%s <empty>", prefix);
        return;
    }

    char hexbuf[3 * 8 + 1]; // hasta 8 bytes: "FF " * 8 + '\0'
    int pos = 0;

    for (uint8_t i = 0; i < len && i < 8; ++i) {
        pos += std::snprintf(hexbuf + pos,
                             sizeof(hexbuf) - pos,
                             (i == len - 1) ? "%02X" : "%02X ",
                             data[i]);
    }

    hexbuf[pos] = '\0';

    log.logInfo("%s %s", prefix, hexbuf);
}

inline bool CAN_Transport::waitWritableUntil(deadline_t deadline) {
    if (!connected()) return false;

    using namespace std::chrono;

    auto now = stdclock::now();
    if (now >= deadline) return false;

    // Tiempo restante hasta el deadline
    auto remaining = duration_cast<microseconds>(deadline - now).count();

    int timeout_ms = static_cast<int>(std::min<long>(remaining, 100)); 

    struct pollfd pfd{ sock_fd_, POLLOUT, 0 };

    int ret = poll(&pfd, 1, timeout_ms);

    if (ret > 0 && (pfd.revents & POLLOUT)) {
        return true;
    }

    return false; // timeout o error
}

// -----------------------------------------------------------------------------
// sendFrame — transmit one raw CAN 2.0B frame (max 8 bytes)
// -----------------------------------------------------------------------------
inline bool CAN_Transport::sendFrame(const uint8_t *frame_data,
                                      uint8_t        frame_len,
                                      deadline_t     deadline) {
    while(true){
    if (stdclock::now() >= deadline) return false;

    if (!waitFdReady(POLLOUT, deadline)) {
        log.logWarn("CAN sendFrame: poll timeout on %s, tx=0x%08X  rx=0x%08X", interface_.c_str(), tx_id_, rx_id_);
        return false;
    }

    struct can_frame frame{};
    frame.can_id  = tx_id_;
    frame.can_dlc = frame_len;
    std::memcpy(frame.data, frame_data, frame_len);

    ssize_t sent = ::write(sock_fd_, &frame, sizeof(struct can_frame));
    //logBufferHex(log, "Sent:", frame_data, frame_len);
    if (sent != static_cast<ssize_t>(sizeof(struct can_frame))) {
        if(errno == EAGAIN || errno == ENOBUFS){
            if (!waitWritableUntil(deadline))
                return false;
            continue;
        }

        log.logError("CAN write() failed on %s: %s", interface_.c_str(), strerror(errno));
        statusReport.with([](Status &d){
          d.level = Status::ERROR;
          d.values["kernel"] = strerror(errno);
        });
        disconnect();
        transport_error_ = Transport_Error::IOCTL_FAILED;
        return false;
    }
    return true;
    }
}

// -----------------------------------------------------------------------------
// recvFrame — receive one raw CAN 2.0B frame, return payload length (0 = fail)
// Frames whose ID doesn't match rx_id_ are silently skipped.
// -----------------------------------------------------------------------------
inline uint8_t CAN_Transport::recvFrame(uint8_t *frame_data, deadline_t deadline) {
    while (true) {
        if (stdclock::now() >= deadline) return 0;
        if (!waitFdReady(POLLIN, deadline))  return 0;

        struct can_frame frame{};
        ssize_t nbytes = ::read(sock_fd_, &frame, sizeof(struct can_frame));
        if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame))) {
            log.logError("CAN read() failed on %s: %s", interface_.c_str(), strerror(errno));
            statusReport.with([](Status &d){
                d.level = Status::ERROR;
                d.values["kernel"] = strerror(errno);
            });
            disconnect();
            transport_error_ = Transport_Error::FD_INVALID;
            return 0;
        }

        // logBufferHex(log, "Read:", frame.data, frame.can_dlc);

        // Double-check ID (hardware filter should already guarantee this)
        if ((frame.can_id & CAN_EFF_MASK) != (rx_id_ & CAN_EFF_MASK)) continue;

        uint8_t dlc = frame.can_dlc;
        if (dlc > CAN_MAX_DLEN) dlc = CAN_MAX_DLEN;
        std::memcpy(frame_data, frame.data, dlc);
        return dlc;
    }
}

// -----------------------------------------------------------------------------
// writeData — segment a buffer and transmit as CAN 2.0B frames
//
// Frame layout:
//   byte 0        : header  [type(2b) | seq(6b)]
//   bytes 1…(dlc-1): payload chunk (up to 7 bytes)
// -----------------------------------------------------------------------------
inline size_t CAN_Transport::writeData(const uint8_t *data, size_t length,
                                        deadline_t deadline) {
    if (!connected() || !data || length == 0) return 0;

    constexpr size_t DLEN = seg::FRAME_DATA_LEN;  // 7
    uint8_t frame_buf[CAN_MAX_DLEN];

    // ── Single frame (fits in 7 data bytes) ──────────────────────────────────
    if (length <= DLEN) {
        frame_buf[0] = seg::make_header(seg::TYPE_SINGLE, tx_seq_);
        tx_seq_      = (tx_seq_ + 1) & 0x3F;
        std::memcpy(frame_buf + 1, data, length);
        return sendFrame(frame_buf, static_cast<uint8_t>(1 + length), deadline)
               ? length : 0;
    }

    // ── Multi-frame ───────────────────────────────────────────────────────────
    size_t offset = 0;
    bool   first  = true;

    while (offset < length) {
        size_t  chunk = std::min(DLEN, length - offset);
        bool    last  = (offset + chunk >= length);
        uint8_t type  = first ? seg::TYPE_START
                      : last  ? seg::TYPE_END
                               : seg::TYPE_CONT;

        frame_buf[0] = seg::make_header(type, tx_seq_);
        tx_seq_      = (tx_seq_ + 1) & 0x3F;
        std::memcpy(frame_buf + 1, data + offset, chunk);

        if (!sendFrame(frame_buf, static_cast<uint8_t>(1 + chunk), deadline))
            return 0;  // abort — caller must retry

        offset += chunk;
        first   = false;
    }

    return length;
}

inline bool CAN_Transport::assemble(deadline_t deadline){
    
    uint8_t frame_buf[CAN_MAX_DLEN];

    uint8_t dlc = recvFrame(frame_buf, deadline);
    if (dlc < 1) return false;  // timeout or socket errors

    uint8_t  header  = frame_buf[0];
    uint8_t  type    = seg::frame_type(header);
    uint8_t  seq     = seg::frame_seq(header);
    uint8_t  datalen = static_cast<uint8_t>(dlc - 1);
    uint8_t *fdata   = frame_buf + 1;

    switch (type) {
    // ── SINGLE: complete message in one frame ─────────────────────────
    case seg::TYPE_SINGLE:
        if (rx_in_progress_)
            log.logWarn("CAN RX: SINGLE mid-transfer, discarding previous");
        rx_in_progress_  = false;   // no multi-frame in progress
        rx_expected_seq_ = (seq + 1) & 0x3F;
        if (datalen > INTERNAL_BUF_SIZE) {
            log.logError("CAN RX: SINGLE frame overflow");
            return false;
        }
        std::memcpy(rx_buf_, fdata, datalen);
        rx_pos_  = datalen;
        read_pos_ = 0;
        return true;

    // ── START: first frame of a multi-frame message ───────────────────
    case seg::TYPE_START:
        if (rx_in_progress_)
            log.logWarn("CAN RX: START during transfer, restarting");
        rx_in_progress_  = true;
        rx_pos_          = 0;
        read_pos_        = 0;
        rx_expected_seq_ = (seq + 1) & 0x3F;
        if (datalen > INTERNAL_BUF_SIZE) {
            log.logError("CAN RX: assembly buffer overflow on START");
            rx_in_progress_ = false;
            return false;
        }
        std::memcpy(rx_buf_, fdata, datalen);
        rx_pos_ = datalen;
        return false;   

    // ── CONT: continuation frame ──────────────────────────────────────
    case seg::TYPE_CONT:
        if (!rx_in_progress_) {
            log.logWarn("CAN RX: CONT without START, discarding");
            return false;
        }
        if (seq != rx_expected_seq_) {
            log.logWarn("CAN RX: seq gap on CONT (exp %u got %u), aborting",
                    rx_expected_seq_, seq);
            rx_in_progress_  = false;
            rx_pos_          = 0;
            transport_error_ = Transport_Error::SEG_ERROR;
            return false;
        }
        rx_expected_seq_ = (seq + 1) & 0x3F;
        if (rx_pos_ + datalen > INTERNAL_BUF_SIZE) {
            log.logError("CAN RX: assembly buffer overflow on CONT");
            rx_in_progress_ = false;
            rx_pos_         = 0;
            return false;
        }
        std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
        rx_pos_ += datalen;
        return false;   

    // ── END: last frame — message complete ───────────────────────────
    case seg::TYPE_END:
        if (!rx_in_progress_) {
            log.logWarn("CAN RX: END without START, discarding");
            return false;
        }
        if (seq != rx_expected_seq_) {
            log.logWarn("CAN RX: seq gap on END (exp %u got %u), aborting",
                    rx_expected_seq_, seq);
            rx_in_progress_  = false;
            rx_pos_          = 0;
            transport_error_ = Transport_Error::SEG_ERROR;
            return false;
        }
        if (rx_pos_ + datalen > INTERNAL_BUF_SIZE) {
            log.logError("CAN RX: assembly buffer overflow on END");
            rx_in_progress_ = false;
            rx_pos_         = 0;
            return false;
        }
        std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
        rx_pos_         += datalen;
        rx_in_progress_  = false;   
        read_pos_        = 0;
        return true; 

    default:
        log.logWarn("CAN RX: unknown frame type %u, skipping", type);
        return false;
    }

    return false;
        
}

// -----------------------------------------------------------------------------
// readData — receive and reassemble segmented CAN 2.0B frames
//
// Reads raw frames via recvFrame() and interprets each segment header.
// SINGLE: copies payload directly to caller's buffer.
// START/CONT/END: assembles into internal rx_buf_, flushes on END.
//
// Returns total bytes written to `buffer`.
// -----------------------------------------------------------------------------
inline size_t CAN_Transport::readData(uint8_t *buffer, size_t length, deadline_t deadline) {
    if (!connected() || !buffer || length == 0) return 0;

    size_t total = 0;

    while (total < length) {
        
        if (read_pos_ >= rx_pos_) {
            read_pos_ = 0;
            rx_pos_   = 0;

            bool complete = false;
            while (!complete) {
                complete = assemble(deadline);
                if (!complete && !rx_in_progress_) {
                    return total;
                }
            }
        }

        size_t available = rx_pos_ - read_pos_;
        size_t to_copy   = std::min(available, length - total);
        std::memcpy(buffer + total, rx_buf_ + read_pos_, to_copy);
        read_pos_ += to_copy; 
        total     += to_copy;
    }

    return total;
}