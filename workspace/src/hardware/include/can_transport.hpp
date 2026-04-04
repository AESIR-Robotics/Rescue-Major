#pragma once

//#include <array>
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
// Bit layout:
//   [28..26] priority  (3 bits, 0=highest, 7=lowest)
//   [25..18] sender    (8 bits, node address 0x00-0xFF)
//   [17..10] receiver  (8 bits, node address 0x00-0xFF)
//   [9..0]   channel   (10 bits, stream channel 0-1023)
//
// Each Protocol_Handler<CAN_Transport> instance is bound to one
// (sender, receiver, channel) tuple. It only reads frames that match
// its rx_id and writes frames with its tx_id.
// =============================================================================

inline constexpr uint32_t can_make_id(uint8_t sender, uint8_t receiver,
                                       uint16_t channel = 0,
                                       uint8_t priority = 6) {
  return CAN_EFF_FLAG |
         ((static_cast<uint32_t>(priority  & 0x07) << 26) |
          (static_cast<uint32_t>(sender              ) << 18) |
          (static_cast<uint32_t>(receiver             ) << 10) |
          (static_cast<uint32_t>(channel   & 0x3FF)        ));
}

// =============================================================================
// CAN_Transport
//
// Owns a SocketCAN file descriptor, connection lifecycle, and CAN-FD I/O.
// Protocol_Handler<CAN_Transport> inherits this and calls:
//   writeData(data, length, deadline)
//   readData(buffer, length, deadline)
//   connected()
//
// Each instance is bound to one logical channel:
//   tx_id  = can_make_id(my_addr, peer_addr, channel)
//   rx_id  = can_make_id(peer_addr, my_addr, channel)
//
// Frames whose ID does not match rx_id are discarded silently by the
// kernel hardware filter — no CPU time is spent on them.
//
// INTERNAL_BUF_SIZE: CAN-FD payloads arrive as complete frames (up to
// 64 bytes). We treat the last received frame as the internal cache,
// so INTERNAL_BUF_SIZE == CANFD_MAX_DLEN.
// =============================================================================

class CAN_Transport {
public:
  enum class Transport_Error {
    NONE,
    CLOSED,
    OPEN_FAILED,     ///< socket() or bind() failed.
    INVALID_CONFIG,  ///< Empty interface or invalid addresses.
    IOCTL_FAILED,    ///< setsockopt / ioctl failed.
    FD_INVALID,      ///< poll returned EBADF — socket dead.
  };

  // CAN-FD max payload per frame — mirrors CANFD_MAX_DLEN in <linux/can.h>
  static constexpr size_t INTERNAL_BUF_SIZE = CANFD_MAX_DLEN; // 64

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

  bool            connected()          const { return sock_fd_ >= 0; }
  const std::string &getDevice()       const { return interface_;    }
  int             getSlaveAddress()    const { return peer_addr_;    }
  Transport_Error getTransportError()  const { return transport_error_; }

  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    log_info_  = std::move(info);
    log_warn_  = std::move(warn);
    log_error_ = std::move(error);
  }

protected:
  // ── Called by Protocol_Handler ─────────────────────────────────────────────

  /// Send `length` bytes as a single CAN-FD frame with tx_id_.
  /// Returns bytes written (0 on failure).
  size_t writeData(const uint8_t *data, size_t length, deadline_t deadline);

  /// Receive bytes into buffer from a CAN-FD frame matching rx_id_.
  /// Frames arrive complete — fills internal_buf_ then copies to caller.
  /// Returns bytes copied (may be < length if frame was shorter).
  size_t readData(uint8_t *buffer, size_t length, deadline_t deadline);

  void disconnect();

  Transport_Error transport_error_ { Transport_Error::CLOSED };

private:
  bool isValidConfig() const noexcept;
  bool waitFdReady(short events, deadline_t deadline);

  std::string interface_;
  uint8_t     my_addr_   { 0x00 };
  uint8_t     peer_addr_ { 0x00 };
  uint16_t    channel_   { 0    };
  uint8_t     priority_  { 6    };

  uint32_t    tx_id_     { 0 };  ///< ID used when sending frames
  uint32_t    rx_id_     { 0 };  ///< ID expected on incoming frames

  int sock_fd_ { -1 };

  // Internal frame buffer — last received CAN-FD payload
  uint8_t internal_buf_[INTERNAL_BUF_SIZE]{};
  size_t  internal_pos_ { INTERNAL_BUF_SIZE }; // start "full" → first read fetches a frame

  LogFn log_info_{};
  LogFn log_warn_{};
  LogFn log_error_{};

  template<typename... Args>
  void logInfo (const char *fmt, Args&&... args) const { logDispatch(log_info_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logWarn (const char *fmt, Args&&... args) const { logDispatch(log_warn_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logError(const char *fmt, Args&&... args) const { logDispatch(log_error_, fmt, std::forward<Args>(args)...); }
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
      channel_{channel}, priority_{priority} {
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

  // Build TX and RX IDs from addressing tuple
  tx_id_ = can_make_id(my_addr_,   peer_addr_, channel_, priority_);
  rx_id_ = can_make_id(peer_addr_, my_addr_,   channel_, priority_);

  // Create RAW CAN socket
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ < 0) {
    transport_error_ = Transport_Error::OPEN_FAILED;
    logError("CAN socket() failed: %s", strerror(errno));
    return false;
  }

  // Enable CAN-FD frames
  int enable = 1;
  if (setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                 &enable, sizeof(enable)) < 0) {
    transport_error_ = Transport_Error::IOCTL_FAILED;
    logError("CAN_RAW_FD_FRAMES setsockopt failed: %s", strerror(errno));
    disconnect();
    return false;
  }

  // Set hardware filter: only receive frames with rx_id_
  // The mask is CAN_EFF_MASK | CAN_EFF_FLAG to match the full 29-bit ID exactly.
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

  // Bind to the CAN interface
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

  // Set non-blocking
  int flags = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

  logInfo("CAN connected on %s tx=0x%08X rx=0x%08X", interface_.c_str(), tx_id_, rx_id_);
  return true;
}

inline void CAN_Transport::disconnect() {
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_      = -1;
    internal_pos_ = INTERNAL_BUF_SIZE;
    logInfo("CAN socket closed on %s tx=0x%08X rx=0x%08X", interface_.c_str(), tx_id_, rx_id_);
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

inline size_t CAN_Transport::writeData(const uint8_t *data, size_t length,
                                        deadline_t deadline) {
  if (!connected() || !data || length == 0 || length > INTERNAL_BUF_SIZE)
    return 0;

  if (stdclock::now() >= deadline) {
    logWarn("CAN send timeout (pre-poll) on %s", interface_.c_str());
    return 0;
  }
  if (!waitFdReady(POLLOUT, deadline)) {
    logWarn("CAN send poll timeout/error on %s", interface_.c_str());
    return 0;
  }

  struct canfd_frame frame{};
  frame.can_id = tx_id_;
  frame.len    = static_cast<uint8_t>(length);
  frame.flags  = CANFD_BRS;  // use higher bitrate for data phase
  std::memcpy(frame.data, data, length);

  ssize_t sent = ::write(sock_fd_, &frame, CANFD_MTU);
  if (sent != CANFD_MTU) {
    logError("CAN write() failed on %s: %s", interface_.c_str(), strerror(errno));
    disconnect();
    transport_error_ = Transport_Error::IOCTL_FAILED;
    return 0;
  }
  return length;
}

inline size_t CAN_Transport::readData(uint8_t *buffer, size_t length,
                                       deadline_t deadline) {
  if (!connected() || !buffer || length == 0) return 0;

  size_t total = 0;

  // ── Step 1: drain internal frame cache ───────────────────────────────────
  if (internal_pos_ < INTERNAL_BUF_SIZE) {
    size_t cached  = INTERNAL_BUF_SIZE - internal_pos_;
    size_t to_copy = std::min(cached, length);
    std::memcpy(buffer, internal_buf_ + internal_pos_, to_copy);
    internal_pos_ += to_copy;
    total         += to_copy;
  }
  if (total == length) return total;

  // ── Step 2: receive next CAN-FD frame ────────────────────────────────────
  // In CAN, each frame arrives complete — no skip protocol needed.
  // We receive one frame at a time and cache the bytes we don't need yet.
  while (total < length) {
    if (stdclock::now() >= deadline) {
      logWarn("CAN read timeout (pre-poll) on %s: expected %zu got %zu",
              interface_.c_str(), length, total);
      break;
    }
    if (!waitFdReady(POLLIN, deadline)) {
      logWarn("CAN read poll timeout/error on %s: expected %zu got %zu",
              interface_.c_str(), length, total);
      break;
    }

    struct canfd_frame frame{};
    ssize_t nbytes = ::read(sock_fd_, &frame, CANFD_MTU);
    if (nbytes != CANFD_MTU) {
      logError("CAN read() failed on %s: %s", interface_.c_str(), strerror(errno));
      disconnect();
      transport_error_ = Transport_Error::FD_INVALID;
      break;
    }

    // Hardware filter guarantees frame.can_id == rx_id_, but double-check.
    if ((frame.can_id & CAN_EFF_MASK) != (rx_id_ & CAN_EFF_MASK)) continue;

    size_t frame_len = frame.len;
    size_t remaining = length - total;

    if (frame_len <= remaining) {
      // Entire frame fits into caller's buffer
      std::memcpy(buffer + total, frame.data, frame_len);
      total        += frame_len;
      internal_pos_ = INTERNAL_BUF_SIZE;  // cache empty
    } else {
      // Frame is larger than what caller needs — cache the tail
      std::memcpy(buffer + total, frame.data, remaining);
      total += remaining;
      std::memcpy(internal_buf_, frame.data, frame_len);
      internal_pos_ = remaining;           // next read starts here
    }
  }

  return total;
}