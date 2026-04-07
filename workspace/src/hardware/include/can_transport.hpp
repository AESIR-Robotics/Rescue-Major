#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
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

using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;
using LogFn      = std::function<void(const std::string &)>;

// =============================================================================
// CAN ID encoding (29-bit extended frame)
//
//   [28..26] priority  (3 bits)
//   [25..18] sender    (8 bits)
//   [17..10] receiver  (8 bits)
//   [9..0]   channel   (10 bits)
// =============================================================================

inline constexpr uint32_t can_make_id(uint8_t sender, uint8_t receiver,
                                       uint16_t channel  = 0,
                                       uint8_t  priority = 6) {
    return CAN_EFF_FLAG |
           ((static_cast<uint32_t>(priority & 0x07) << 26) |
            (static_cast<uint32_t>(sender           ) << 18) |
            (static_cast<uint32_t>(receiver          ) << 10) |
            (static_cast<uint32_t>(channel  & 0x3FF) ));
}

// =============================================================================
// CAN 2.0B Segmentation Protocol
//
// CAN 2.0B max payload = 8 bytes per frame.
// Byte 0 is a segment header; bytes 1-7 carry up to 7 bytes of data.
//
// Segment header (byte 0):
//   [7..6]  frame type:
//             0b00 (SINGLE) — entire message fits in one frame
//             0b01 (START)  — first frame of a multi-frame message
//             0b10 (CONT)   — continuation frame
//             0b11 (END)    — last frame of a multi-frame message
//   [5..0]  sequence number (0-63, wraps)
//
// The receiver reassembles frames in order. If a sequence gap or timeout
// occurs the partial transfer is discarded.
// =============================================================================

namespace seg {
    constexpr uint8_t HEADER_SIZE    = 1;
    constexpr uint8_t FRAME_DATA_LEN = CAN_MAX_DLEN - HEADER_SIZE;  // 7

    constexpr uint8_t TYPE_SINGLE = 0b00;
    constexpr uint8_t TYPE_START  = 0b01;
    constexpr uint8_t TYPE_CONT   = 0b10;
    constexpr uint8_t TYPE_END    = 0b11;

    inline uint8_t make_header(uint8_t type, uint8_t seq) {
        return static_cast<uint8_t>((type & 0x03) << 6) | (seq & 0x3F);
    }
    inline uint8_t frame_type(uint8_t header) { return (header >> 6) & 0x03; }
    inline uint8_t frame_seq (uint8_t header) { return  header       & 0x3F; }
} // namespace seg

// =============================================================================
// CAN_Transport
//
// Owns a SocketCAN file descriptor and provides CAN 2.0B segmented I/O.
// Public/protected interface is identical to the original so that
// Protocol_Handler<CAN_Transport> compiles without changes.
// =============================================================================

class CAN_Transport {
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
    static constexpr size_t INTERNAL_BUF_SIZE = 256;

    explicit CAN_Transport(const std::string &interface = "",
                           uint8_t  my_addr   = 0x00,
                           uint8_t  peer_addr = 0x00,
                           uint16_t channel   = 0,
                           uint8_t  priority  = 6);
    ~CAN_Transport() noexcept;

    CAN_Transport(const CAN_Transport &)            = delete;
    CAN_Transport &operator=(const CAN_Transport &) = delete;

    bool init(const std::string &interface,
              uint8_t  my_addr,
              uint8_t  peer_addr,
              uint16_t channel  = 0,
              uint8_t  priority = 6);
    bool connect();
    bool reconnect();

    bool               connected()         const { return sock_fd_ >= 0;   }
    const std::string &getDevice()         const { return interface_;       }
    int                getSlaveAddress()   const { return peer_addr_;       }
    Transport_Error    getTransportError() const { return transport_error_; }

    void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
        log_info_  = std::move(info);
        log_warn_  = std::move(warn);
        log_error_ = std::move(error);
    }

protected:
    // ── Called by Protocol_Handler ──────────────────────────────────────────

    /// Segment `data` into CAN 2.0B frames and transmit all of them.
    /// Returns bytes written (0 on failure).
    size_t writeData(const uint8_t *data, size_t length, deadline_t deadline);

    bool hasData();

    /// Receive and reassemble segmented CAN 2.0B frames into `buffer`.
    /// Returns bytes copied.
    size_t readData(uint8_t *buffer, size_t length, deadline_t deadline);

    void disconnect();

    Transport_Error transport_error_ { Transport_Error::CLOSED };

private:
    // ── Segmentation helpers ────────────────────────────────────────────────
    bool    sendFrame(const uint8_t *frame_data, uint8_t frame_len,
                      deadline_t deadline);
    uint8_t recvFrame(uint8_t *frame_data, deadline_t deadline);

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

    // TX: 6-bit rolling sequence counter
    uint8_t     tx_seq_    { 0 };

    // RX reassembly state
    uint8_t     rx_buf_[INTERNAL_BUF_SIZE]{};
    size_t      rx_pos_          { 0     };
    uint8_t     rx_expected_seq_ { 0     };
    bool        rx_in_progress_  { false };

    LogFn log_info_{};
    LogFn log_warn_{};
    LogFn log_error_{};

    template<typename... Args>
    void logInfo(const char *fmt, Args&&... args) const {
        logDispatch(log_info_, fmt, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void logWarn(const char *fmt, Args&&... args) const {
        logDispatch(log_warn_, fmt, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void logError(const char *fmt, Args&&... args) const {
        logDispatch(log_error_, fmt, std::forward<Args>(args)...);
    }
    template<typename... Args>
    static void logDispatch(const LogFn &fn, const char *fmt, Args&&... args) {
        if (!fn) return;
        char buf[256];
        std::snprintf(buf, sizeof(buf), fmt, std::forward<Args>(args)...);
        fn(buf);
    }
};

// =============================================================================
// Inline definitions
// =============================================================================

inline CAN_Transport::CAN_Transport(const std::string &interface,
                                     uint8_t my_addr, uint8_t peer_addr,
                                     uint16_t channel, uint8_t priority)
    : interface_{interface}, my_addr_{my_addr}, peer_addr_{peer_addr},
      channel_{channel}, priority_{priority}
{
    if (!interface_.empty()) connect();
}

inline CAN_Transport::~CAN_Transport() noexcept { disconnect(); }

inline bool CAN_Transport::init(const std::string &interface,
                                 uint8_t my_addr, uint8_t peer_addr,
                                 uint16_t channel, uint8_t priority) {
    interface_ = interface;
    my_addr_   = my_addr;
    peer_addr_ = peer_addr;
    channel_   = channel;
    priority_  = priority;
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
    disconnect();
    transport_error_ = Transport_Error::NONE;

    if (!isValidConfig()) {
        transport_error_ = Transport_Error::INVALID_CONFIG;
        return false;
    }

    tx_id_ = can_make_id(my_addr_,   peer_addr_, channel_, priority_);
    rx_id_ = can_make_id(peer_addr_, my_addr_,   channel_, priority_);

    // Classic CAN 2.0B socket — do NOT enable CAN_RAW_FD_FRAMES
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        logError("CAN socket() failed: %s", strerror(errno));
        return false;
    }

    // Hardware filter: only receive frames matching rx_id_
    struct can_filter rfilter{};
    rfilter.can_id   = rx_id_;
    rfilter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
    if (setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
                   &rfilter, sizeof(rfilter)) < 0) {
        transport_error_ = Transport_Error::IOCTL_FAILED;
        logError("CAN_RAW_FILTER setsockopt failed: %s", strerror(errno));
        disconnect();
        return false;
    }

    // Bind to interface
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        logError("CAN interface %s not found: %s", interface_.c_str(), strerror(errno));
        disconnect();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        transport_error_ = Transport_Error::OPEN_FAILED;
        logError("CAN bind() failed: %s", strerror(errno));
        disconnect();
        return false;
    }

    // Non-blocking I/O
    int flags = fcntl(sock_fd_, F_GETFL, 0);
    fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

    // Reset segmentation state
    tx_seq_         = 0;
    rx_pos_         = 0;
    rx_in_progress_ = false;

    logInfo("CAN 2.0B connected on %s  tx=0x%08X  rx=0x%08X",
            interface_.c_str(), tx_id_, rx_id_);
    return true;
}

inline void CAN_Transport::disconnect() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_        = -1;
        rx_pos_         = 0;
        rx_in_progress_ = false;
        logInfo("CAN socket closed on %s", interface_.c_str());
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
        }
        return false;
    }
    return ret > 0 && (pfd.revents & events);
}

inline bool CAN_Transport::hasData() { return true; }

// -----------------------------------------------------------------------------
// sendFrame — transmit one raw CAN 2.0B frame (max 8 bytes)
// -----------------------------------------------------------------------------
inline bool CAN_Transport::sendFrame(const uint8_t *frame_data,
                                      uint8_t        frame_len,
                                      deadline_t     deadline) {
    if (stdclock::now() >= deadline) return false;

    if (!waitFdReady(POLLOUT, deadline)) {
        logWarn("CAN sendFrame: poll timeout on %s", interface_.c_str());
        return false;
    }

    struct can_frame frame{};
    frame.can_id  = tx_id_;
    frame.can_dlc = frame_len;
    std::memcpy(frame.data, frame_data, frame_len);

    ssize_t sent = ::write(sock_fd_, &frame, sizeof(struct can_frame));
    if (sent != static_cast<ssize_t>(sizeof(struct can_frame))) {
        logError("CAN write() failed on %s: %s", interface_.c_str(), strerror(errno));
        disconnect();
        transport_error_ = Transport_Error::IOCTL_FAILED;
        return false;
    }
    return true;
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
            logError("CAN read() failed on %s: %s", interface_.c_str(), strerror(errno));
            disconnect();
            transport_error_ = Transport_Error::FD_INVALID;
            return 0;
        }

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

// -----------------------------------------------------------------------------
// readData — receive and reassemble segmented CAN 2.0B frames
//
// Reads raw frames via recvFrame() and interprets each segment header.
// SINGLE: copies payload directly to caller's buffer.
// START/CONT/END: assembles into internal rx_buf_, flushes on END.
//
// Returns total bytes written to `buffer`.
// -----------------------------------------------------------------------------
inline size_t CAN_Transport::readData(uint8_t *buffer, size_t length,
                                       deadline_t deadline) {
    if (!connected() || !buffer || length == 0) return 0;

    uint8_t frame_buf[CAN_MAX_DLEN];
    size_t  total = 0;

    while (total < length) {
        uint8_t dlc = recvFrame(frame_buf, deadline);
        if (dlc < 1) break;  // timeout or socket error

        uint8_t  header  = frame_buf[0];
        uint8_t  type    = seg::frame_type(header);
        uint8_t  seq     = seg::frame_seq(header);
        uint8_t  datalen = static_cast<uint8_t>(dlc - 1);
        uint8_t *fdata   = frame_buf + 1;

        switch (type) {

        // ── SINGLE: complete message in one frame ─────────────────────────
        case seg::TYPE_SINGLE:
            if (rx_in_progress_) {
                logWarn("CAN RX: SINGLE mid-transfer, discarding previous");
                rx_in_progress_ = false;
                rx_pos_         = 0;
            }
            rx_expected_seq_ = (seq + 1) & 0x3F;
            {
                size_t to_copy = std::min(static_cast<size_t>(datalen), length - total);
                std::memcpy(buffer + total, fdata, to_copy);
                total += to_copy;
            }
            return total;

        // ── START: first frame of a multi-frame message ───────────────────
        case seg::TYPE_START:
            if (rx_in_progress_)
                logWarn("CAN RX: START during transfer, restarting");
            rx_in_progress_  = true;
            rx_pos_          = 0;
            rx_expected_seq_ = (seq + 1) & 0x3F;

            if (rx_pos_ + datalen <= INTERNAL_BUF_SIZE) {
                std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
                rx_pos_ += datalen;
            } else {
                logError("CAN RX: assembly buffer overflow on START");
                rx_in_progress_ = false;
                rx_pos_         = 0;
            }
            break;

        // ── CONT: continuation frame ──────────────────────────────────────
        case seg::TYPE_CONT:
            if (!rx_in_progress_) {
                logWarn("CAN RX: CONT without START, discarding");
                break;
            }
            if (seq != rx_expected_seq_) {
                logWarn("CAN RX: seq gap on CONT (exp %u got %u), aborting",
                        rx_expected_seq_, seq);
                rx_in_progress_  = false;
                rx_pos_          = 0;
                transport_error_ = Transport_Error::SEG_ERROR;
                return total;
            }
            rx_expected_seq_ = (seq + 1) & 0x3F;

            if (rx_pos_ + datalen <= INTERNAL_BUF_SIZE) {
                std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
                rx_pos_ += datalen;
            } else {
                logError("CAN RX: assembly buffer overflow on CONT");
                rx_in_progress_ = false;
                rx_pos_         = 0;
                return total;
            }
            break;

        // ── END: last frame — flush assembled message ─────────────────────
        case seg::TYPE_END:
            if (!rx_in_progress_) {
                logWarn("CAN RX: END without START, discarding");
                break;
            }
            if (seq != rx_expected_seq_) {
                logWarn("CAN RX: seq gap on END (exp %u got %u), aborting",
                        rx_expected_seq_, seq);
                rx_in_progress_  = false;
                rx_pos_          = 0;
                transport_error_ = Transport_Error::SEG_ERROR;
                return total;
            }
            rx_expected_seq_ = (seq + 1) & 0x3F;

            if (rx_pos_ + datalen <= INTERNAL_BUF_SIZE) {
                std::memcpy(rx_buf_ + rx_pos_, fdata, datalen);
                rx_pos_ += datalen;
            } else {
                logError("CAN RX: assembly buffer overflow on END");
                rx_in_progress_ = false;
                rx_pos_         = 0;
                return total;
            }

            // Deliver assembled message to caller
            {
                size_t to_copy = std::min(rx_pos_, length - total);
                std::memcpy(buffer + total, rx_buf_, to_copy);
                total          += to_copy;
                rx_pos_         = 0;
                rx_in_progress_ = false;
            }
            return total;

        default:
            logWarn("CAN RX: unknown frame type %u, skipping", type);
            break;
        }
    }

    return total;
}