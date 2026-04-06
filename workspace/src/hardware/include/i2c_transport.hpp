#pragma once

//#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/file.h>
#include <cerrno>
#include <poll.h>
#include <unistd.h>

#include "crc.hpp"

using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;
using LogFn      = std::function<void(const std::string &)>;

class I2C_Transport {
public:
  enum class Transport_Error {
    NONE,          ///< Connected and operating normally.
    CLOSED,        ///< Closed cleanly — ready for reconnect().
    OPEN_FAILED,   ///< open() failed — device path may be invalid.
    INVALID_CONFIG,///< Empty device string or slave_addr out of 0x03-0x77.
    DEVICE_BUSY,   ///< Another process holds the lock — retry later.
    LOCK_FAILED,   ///< flock() failed for an unexpected reason.
    IOCTL_FAILED,  ///< ioctl returned error — driver rejected the command, fd closed.
    FD_INVALID,    ///< Kernel reports fd invalid (EBADF) — fd closed.
  };

  explicit I2C_Transport(const std::string &device_in = "",
                         int slave_addr_in = 0x00);
  ~I2C_Transport() noexcept;

  // Non-copyable — owns a file descriptor.
  I2C_Transport(const I2C_Transport &)            = delete;
  I2C_Transport &operator=(const I2C_Transport &) = delete;

  bool init(const std::string &device_in, int slave_addr_in);
  bool connect();
  bool reconnect();
  

  bool                connected()       const { return i2c_fd >= 0; }
  const std::string & getDevice()      const { return device;     }
  int                 getSlaveAddress() const { return slave_addr; }
  Transport_Error     getTransportError() const { return transport_error_; }

  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    log_info_  = std::move(info);
    log_warn_  = std::move(warn);
    log_error_ = std::move(error);
  }

  // ── Internal buffer constants ─────────────────────────────────────────────
  // WARNING: do not set INTERNAL_BUF_SIZE to 3 — collides with the no-message
  // sentinel frame (0xAA 0x00 CRC).
  static constexpr size_t INTERNAL_BUF_SIZE = 8;
  static_assert(INTERNAL_BUF_SIZE != 3,
      "INTERNAL_BUF_SIZE == 3 collides with the no-message sentinel frame");

protected:
  // ── Low-level I/O — called by Protocol_Handler, not by external code ──────

  /// Write `length` bytes in a single ioctl. Returns bytes written (0 on fail).
  size_t writeData(const uint8_t *data, size_t length, deadline_t deadline);

  bool hasData();
  /// Read `length` bytes using the internal cache + skip protocol.
  /// Returns bytes read (may be < length on timeout or bus error).
  size_t readData(uint8_t *buffer, size_t length, deadline_t deadline);

  void disconnect();

  // Exposed to Protocol_Handler so it can set transport error on protocol faults
  // that imply a transport problem (e.g. partial read after an ioctl failure).
  Transport_Error transport_error_ { Transport_Error::CLOSED };

private:
  bool isValidConfig() const noexcept;
  bool waitFdReady(short events, deadline_t deadline);

  std::string device;
  int i2c_fd    { -1 };
  int slave_addr{ -1 };

  uint8_t internal_buf[INTERNAL_BUF_SIZE]{};
  size_t  internal_pos { INTERNAL_BUF_SIZE }; // start "full" → first read fetches a chunk

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

inline I2C_Transport::I2C_Transport(const std::string &device_in, int slave_addr_in)
    : device{device_in}, slave_addr{slave_addr_in} {
  if (!device.empty()) connect();
}

inline I2C_Transport::~I2C_Transport() noexcept { disconnect(); }

inline bool I2C_Transport::init(const std::string &device_in, int slave_addr_in) {
  device     = device_in;
  slave_addr = slave_addr_in;
  return connect();
}

inline bool I2C_Transport::reconnect() {
  if (device.empty()) {
    transport_error_ = Transport_Error::INVALID_CONFIG;
    return false;
  }
  return connect();
}

inline bool I2C_Transport::connect() {
  disconnect();
  transport_error_ = Transport_Error::NONE;
  if (!isValidConfig()) {
    transport_error_ = Transport_Error::INVALID_CONFIG;
    return false;
  }
  i2c_fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (i2c_fd < 0) {
    transport_error_ = Transport_Error::OPEN_FAILED;
    return false;
  }

  if (flock(i2c_fd, LOCK_EX | LOCK_NB) != 0) {
    if (errno == EWOULDBLOCK) {
      transport_error_ = Transport_Error::DEVICE_BUSY;
      logError("Device is already in use");
    } else {
      transport_error_ = Transport_Error::LOCK_FAILED;
      logError("Failed to lock file");
    }
    disconnect();
    return false;
  }

  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    transport_error_ = Transport_Error::IOCTL_FAILED;
    disconnect();
    return false;
  }

  logInfo("Managed to connect %s at %d", device.c_str(), slave_addr);
  return true;
}

inline void I2C_Transport::disconnect() {
  if (i2c_fd >= 0) {
    ::close(i2c_fd);
    i2c_fd = -1;
    internal_pos = INTERNAL_BUF_SIZE;  // invalidate cache
    logInfo("Closed connection to %s at %d", device.c_str(), slave_addr);
    if (transport_error_ == Transport_Error::NONE)
      transport_error_ = Transport_Error::CLOSED;
  }
}

inline bool I2C_Transport::isValidConfig() const noexcept {
  return !device.empty() && slave_addr >= 0x03 && slave_addr <= 0x77;
}

inline bool I2C_Transport::waitFdReady(short events, deadline_t deadline) {
  struct pollfd pfd{};
  pfd.fd     = i2c_fd;
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

inline bool I2C_Transport::hasData(){return true;}

inline size_t I2C_Transport::writeData(const uint8_t *data, size_t length, 
                                        deadline_t deadline) {
  if (!connected() || length == 0) return 0;

  if (stdclock::now() >= deadline) {
    logWarn("Send timeout (pre-poll) from device %s at address %d (write)",
            device.c_str(), slave_addr);
    return 0;
  }
  if (!waitFdReady(POLLOUT, deadline)) {
    logWarn("Send poll timeout/error from device %s at address %d (write)",
            device.c_str(), slave_addr);
    return 0;
  }

  struct i2c_msg msg{};
  msg.addr  = static_cast<__u16>(slave_addr);
  msg.flags = 0;
  msg.len   = static_cast<__u16>(length);
  msg.buf   = const_cast<uint8_t *>(data);

  struct i2c_rdwr_ioctl_data iod{};
  iod.msgs  = &msg;
  iod.nmsgs = 1;

  if (ioctl(i2c_fd, I2C_RDWR, &iod) != static_cast<int>(iod.nmsgs)) {
    disconnect();
    transport_error_ = Transport_Error::IOCTL_FAILED;
    return 0;
  }
  return length;
}

inline size_t I2C_Transport::readData(uint8_t *buffer, size_t length, 
                                       deadline_t deadline) {
  if (!connected() || !buffer || length == 0) return 0;

  size_t total = 0;

  // ── Step 1: drain internal cache ─────────────────────────────────────────
  if (internal_pos < INTERNAL_BUF_SIZE) {
    size_t cached  = INTERNAL_BUF_SIZE - internal_pos;
    size_t to_copy = std::min(cached, length);
    std::memcpy(buffer, internal_buf + internal_pos, to_copy);
    internal_pos += to_copy;
    total        += to_copy;
  }
  if (total == length) return total;

  // ── Step 2: fetch remaining bytes ────────────────────────────────────────
  size_t remaining = length - total;

  auto doFetch = [&](uint8_t *dest, size_t fetchSize) -> bool {
    if (stdclock::now() >= deadline) {
      logWarn("Read timeout (pre-poll) from device %s at address %d: "
              "expected %zu bytes, got %zu (read)",
              device.c_str(), slave_addr, length, total);
      return false;
    }
    if (!waitFdReady(POLLIN, deadline)) {
      logWarn("Read poll timeout/error from device %s at address %d: "
              "expected %zu bytes, got %zu (read)",
              device.c_str(), slave_addr, length, total);
      return false;
    }

    struct i2c_msg msgs[2]{};
    msgs[0].addr  = static_cast<__u16>(slave_addr);
    msgs[0].flags = I2C_M_RD;
    msgs[0].len   = static_cast<__u16>(fetchSize);
    msgs[0].buf   = dest;

    // Skip: tell the MCU how many bytes were consumed.
    // Two separate ioctls instead of repeated-start due to driver compatibility.
    uint8_t skipmsg[4]{0xBB, 0xFF, static_cast<uint8_t>(fetchSize), 0x00};
    skipmsg[3] = calcCRC(skipmsg, 3, 0);
    msgs[1].addr  = static_cast<__u16>(slave_addr);
    msgs[1].flags = 0;
    msgs[1].len   = 4;
    msgs[1].buf   = skipmsg;

    struct i2c_rdwr_ioctl_data iod{};
    iod.nmsgs = 1;
    for (size_t i = 0; i < 2; ++i) {
      iod.msgs = &msgs[i];
      if (ioctl(i2c_fd, I2C_RDWR, &iod) != static_cast<int>(iod.nmsgs)) {
        logError("I2C ioctl failed: %s", strerror(errno));
        disconnect();
        transport_error_ = Transport_Error::IOCTL_FAILED;
        return false;
      }
    }
    return true;
  };

  if (remaining <= INTERNAL_BUF_SIZE) {
    if (!doFetch(internal_buf, INTERNAL_BUF_SIZE)) return total;
    internal_pos = 0;
    std::memcpy(buffer + total, internal_buf, remaining);
    internal_pos = remaining;
    total        = length;
  } else {
    // Large request — fetch INTERNAL_BUF_SIZE at a time directly into caller's buffer.
    while (remaining > INTERNAL_BUF_SIZE) {
      if (!doFetch(buffer + total, INTERNAL_BUF_SIZE)) return total;
      total     += INTERNAL_BUF_SIZE;
      remaining -= INTERNAL_BUF_SIZE;
    }
    if (remaining > 0) {
      if (!doFetch(internal_buf, INTERNAL_BUF_SIZE)) return total;
      internal_pos = 0;
      std::memcpy(buffer + total, internal_buf, remaining);
      internal_pos = remaining;
      total        = length;
    }
  }

  return total;
}
