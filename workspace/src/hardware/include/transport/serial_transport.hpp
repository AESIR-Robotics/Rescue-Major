#pragma once

//#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>

#include "utils/logger.hpp"
#include "utils/diagnostics.hpp"

using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;
using LogFn      = std::function<void(const std::string &)>;

// =============================================================================
// Serial_Transport
//
// Currently unused, statusReport is unoperative
//
// Owns a UART file descriptor and all serial-specific I/O.
// Protocol_Handler<Serial_Transport> inherits this and calls:
//   writeData(data, length, deadline)   → single write() syscall
//   readData(buffer, length, deadline)  → buffered reads
//   connected()                         → fd is open
//   getTransportError()                 → last transport-level error
//
// Unlike I2C there is no chunk/skip protocol — UART is a true byte stream.
// The internal buffer minimises read() syscalls for single-byte accesses
// (e.g. the sync scan in readPending) without imposing a fixed transfer size
// on the MCU side.
// =============================================================================

class Serial_Transport {
public:
  enum class Transport_Error {
    NONE,
    CLOSED,          ///< Closed cleanly — ready for reconnect().
    OPEN_FAILED,     ///< open() failed — port path may be invalid.
    INVALID_CONFIG,  ///< Empty path or unsupported baud rate.
    CONFIG_FAILED,   ///< termios configuration failed.
    LOCK_FAILED,     ///< flock() failed for an unexpected reason.
    DEVICE_BUSY,     ///< Another process holds the port lock.
    IO_FAILED,       ///< read() or write() returned an error — fd closed.
    FD_INVALID,      ///< poll returned EBADF — fd dead.
  };

  // Internal read buffer — tunable; does not need to match anything on the MCU.
  static constexpr size_t INTERNAL_BUF_SIZE = 64;

  explicit Serial_Transport(DiagnosticRegistry *reg, const std::string &port    = "",
                             uint32_t           baud    = 115200,
                             bool               hw_flow = false);
  virtual ~Serial_Transport() noexcept;

  Serial_Transport(const Serial_Transport &)            = delete;
  Serial_Transport &operator=(const Serial_Transport &) = delete;

  bool init(DiagnosticRegistry *reg, const std::string &port,
            uint32_t           baud    = 115200,
            bool               hw_flow = false);
  bool connect();
  bool reconnect();

  bool               connected()         const { return fd_ >= 0;          }
  const std::string &getDevice()         const { return port_;             }
  int                getSlaveAddress()   const { return 0;                 } // unused for serial
  Transport_Error    getTransportError() const { return transport_error_;  }

  void setLogger(Logger &in_log) {
    log = in_log;
  }

protected:
  size_t writeData(const uint8_t *data, size_t length, deadline_t deadline);
  size_t readData (uint8_t *buffer, size_t length, deadline_t deadline);

  bool canSend();
  bool hasData();

  void disconnect();

  Transport_Error transport_error_ { Transport_Error::CLOSED };

  Tracked<Status> statusReport;

private:
  bool isValidConfig() const noexcept;
  bool applyTermios()  const noexcept;
  bool waitFdReady(short events, deadline_t deadline);

  static speed_t baudToSpeed(uint32_t baud) noexcept;

  std::string port_;
  uint32_t    baud_    { 115200 };
  bool        hw_flow_ { false  };

  int    fd_           { -1 };

  uint8_t internal_buf_[INTERNAL_BUF_SIZE]{};
  size_t  internal_pos_ { INTERNAL_BUF_SIZE }; // start "full" → first read fetches bytes

  Logger log{};
};

// =============================================================================
// Inline definitions
// =============================================================================

inline Serial_Transport::Serial_Transport(DiagnosticRegistry *reg, const std::string &port,
                                           uint32_t baud, bool hw_flow)
    : port_{port}, baud_{baud}, hw_flow_{hw_flow} {

  statusReport.with([this](Status &d){
    d.hardware_id = port_;
    d.level = Status::OK;
    d.message = "The status of the Serial port";
    d.name = "Serial_Transport";
    d.values.emplace(
      std::make_pair("attmps", "0")
    );
  });
  if(reg){
    reg->register_source(&statusReport);
  }
  if (!port_.empty()) connect();
}

inline Serial_Transport::~Serial_Transport() noexcept { disconnect(); }

inline bool Serial_Transport::init(DiagnosticRegistry *reg, const std::string &port,
                                    uint32_t baud, bool hw_flow) {
  if(reg){
    reg->register_source(&statusReport);
  }
  port_    = port;
  baud_    = baud;
  hw_flow_ = hw_flow;
  return connect();
}

inline bool Serial_Transport::reconnect() {
  if (port_.empty()) {
    transport_error_ = Transport_Error::INVALID_CONFIG;
    return false;
  }
  return connect();
}

inline bool Serial_Transport::connect() {
  disconnect();
  transport_error_ = Transport_Error::NONE;

  if (!isValidConfig()) {
    transport_error_ = Transport_Error::INVALID_CONFIG;
    return false;
  }

  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    transport_error_ = Transport_Error::OPEN_FAILED;
    log.logError("Serial open() failed on %s: %s", port_.c_str(), strerror(errno));
    return false;
  }

  // Exclusive lock — same rationale as I2C_Transport.
  if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    if (errno == EWOULDBLOCK) {
      transport_error_ = Transport_Error::DEVICE_BUSY;
      log.logError("Serial port %s already in use", port_.c_str());
    } else {
      transport_error_ = Transport_Error::LOCK_FAILED;
      log.logError("Serial flock() failed on %s: %s", port_.c_str(), strerror(errno));
    }
    disconnect();
    return false;
  }

  if (!applyTermios()) {
    transport_error_ = Transport_Error::CONFIG_FAILED;
    log.logError("Serial termios config failed on %s", port_.c_str());
    disconnect();
    return false;
  }

  log.logInfo("Serial connected on %s at %u baud", port_.c_str(), baud_);
  return true;
}

inline bool Serial_Transport::applyTermios() const noexcept {
  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) return false;

  speed_t speed = baudToSpeed(baud_);
  if (speed == B0) return false;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no echo, no canonical, raw mode
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;    // enable receiver, ignore modem lines
  tty.c_cflag &= ~CSTOPB;           // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;          // no hardware flow control by default

  if (hw_flow_)
    tty.c_cflag |= CRTSCTS;

  // Non-blocking reads — timing is managed by ppoll in waitFdReady
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

inline speed_t Serial_Transport::baudToSpeed(uint32_t baud) noexcept {
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default:      return B0;  // invalid
  }
}

inline void Serial_Transport::disconnect() {
  if (fd_ >= 0) {
    tcdrain(fd_);   // flush pending TX bytes before closing
    ::close(fd_);
    fd_           = -1;
    internal_pos_ = INTERNAL_BUF_SIZE;
    log.logInfo("Serial port %s closed", port_.c_str());
    if (transport_error_ == Transport_Error::NONE)
      transport_error_ = Transport_Error::CLOSED;
  }
}

inline bool Serial_Transport::isValidConfig() const noexcept {
  return !port_.empty() && baudToSpeed(baud_) != B0;
}

inline bool Serial_Transport::waitFdReady(short events, deadline_t deadline) {
  struct pollfd pfd{};
  pfd.fd     = fd_;
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


inline bool Serial_Transport::canSend() {
    if (!connected()) return false;

    struct pollfd pfd{ fd_, POLLOUT, 0 };

    // timeout = 0 → no bloquea
    int ret = poll(&pfd, 1, 0);

    return ret > 0 && (pfd.revents & POLLOUT);
}

inline bool Serial_Transport::hasData() {
    if (!connected()) return false;
    // If there are already assembled bytes ready to read, return immediately
    if (internal_pos_ < INTERNAL_BUF_SIZE) return true;
    // Non-blocking poll — 0 timeout means return instantly
    struct pollfd pfd{ fd_, POLLIN, 0 };
    int ret = poll(&pfd, 1, 0);
    return ret > 0 && (pfd.revents & POLLIN);
}

inline size_t Serial_Transport::writeData(const uint8_t *data, size_t length,
                                           deadline_t deadline) {
  if (!connected() || !data || length == 0) return 0;

  if (stdclock::now() >= deadline) {
    log.logWarn("Serial send timeout (pre-poll) on %s", port_.c_str());
    return 0;
  }
  if (!waitFdReady(POLLOUT, deadline)) {
    log.logWarn("Serial send poll timeout/error on %s", port_.c_str());
    return 0;
  }

  // Serial is a byte stream — write the entire frame in one call.
  ssize_t sent = ::write(fd_, data, length);
  if (sent < 0) {
    log.logError("Serial write() failed on %s: %s", port_.c_str(), strerror(errno));
    disconnect();
    transport_error_ = Transport_Error::IO_FAILED;
    return 0;
  }
  return static_cast<size_t>(sent);
}

inline size_t Serial_Transport::readData(uint8_t *buffer, size_t length,
                                          deadline_t deadline) {
  if (!connected() || !buffer || length == 0) return 0;

  size_t total = 0;

  // ── Step 1: drain internal cache ─────────────────────────────────────────
  if (internal_pos_ < INTERNAL_BUF_SIZE) {
    size_t cached  = INTERNAL_BUF_SIZE - internal_pos_;
    size_t to_copy = std::min(cached, length);
    std::memcpy(buffer, internal_buf_ + internal_pos_, to_copy);
    internal_pos_ += to_copy;
    total         += to_copy;
  }
  if (total == length) return total;

  // ── Step 2: fetch remaining bytes ─────────────────────────────────────────
  // Unlike I2C there is no chunk size constraint from the MCU — we read
  // whatever is available up to INTERNAL_BUF_SIZE at a time to batch syscalls,
  // then copy what the caller needs and keep the rest in cache.
  size_t remaining = length - total;

  while (remaining > 0) {
    if (stdclock::now() >= deadline) {
      log.logWarn("Serial read timeout (pre-poll) on %s: expected %zu got %zu",
              port_.c_str(), length, total);
      break;
    }
    if (!waitFdReady(POLLIN, deadline)) {
      log.logWarn("Serial read poll timeout/error on %s: expected %zu got %zu",
              port_.c_str(), length, total);
      break;
    }

    // Read up to INTERNAL_BUF_SIZE bytes at once to minimise syscalls.
    ssize_t got = ::read(fd_, internal_buf_, INTERNAL_BUF_SIZE);
    if (got < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue; // spurious wakeup
      log.logError("Serial read() failed on %s: %s", port_.c_str(), strerror(errno));
      disconnect();
      transport_error_ = Transport_Error::IO_FAILED;
      break;
    }
    if (got == 0) continue; // no bytes yet despite poll saying ready

    internal_pos_ = 0;
    size_t available = static_cast<size_t>(got);
    size_t to_copy   = std::min(available, remaining);

    std::memcpy(buffer + total, internal_buf_, to_copy);
    internal_pos_ = to_copy;       // next readData call drains from here
    total        += to_copy;
    remaining    -= to_copy;

    // If the batch brought more bytes than needed, they stay in cache.
    // Advance internal_pos_ so only the unconsumed tail is cached.
    if (to_copy < available) {
      // internal_pos_ already == to_copy — the tail [to_copy..available) is cached
      // We need to compact: move the tail to the front of internal_buf_.
      // Actually internal_buf_ already has the full batch; internal_pos_ points
      // to the start of the unconsumed portion — correct as-is.
      break;  // caller got all it needed
    }
  }

  return total;
}