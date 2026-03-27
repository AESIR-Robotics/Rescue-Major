#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
//#include <mutex>
#include <queue>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/file.h>
#include <cerrno>
#include <poll.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Optional logging callbacks — connect to your logger via setLogger().
// If not set all log output is suppressed — no ROS dependency required.
// ---------------------------------------------------------------------------
#include "commands.hpp"
#include "crc.hpp"
#include "tuple_utils.hpp"

using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;  ///< Absolute deadline used internally.

// Log callback type: receives a pre-formatted message string.
using LogFn = std::function<void(const std::string &)>;

// =============================================================================
// Declaration
// =============================================================================
std::string bytes_to_hex_string(const uint8_t* buf, size_t size);

class Protocol_Handler_I2C {
public:
  enum class Error_State {
    NONE,             

    CLOSED,          
    OPEN_FAILED,     

    INVALID_CONFIG,   

    DEVICE_BUSY,      ///< Another process holds the lock — retry later.
    LOCK_FAILED,      ///< flock() failed for an unexpected reason.

    IOCTL_FAILED,     ///< ioctl returned error — driver rejected the command, fd closed.
    IO_ERROR,         ///< Protocol-level fault (bad length, partial read, etc.).
                      ///< The fd remains open — the bus is still alive.
                      ///< Mutually exclusive with any closed/disconnected state.
                      ///< Clear with clearError() once the condition is handled.
    FD_INVALID,       ///< Kernel reports fd invalid (EBADF) — fd closed.
  };

  using header = std::tuple<uint8_t, uint8_t, uint8_t>; // sync, instruction, length
  using tail   = std::tuple<uint8_t>;                    // checksum

  // callbacks must be non-blocking and complete in O(1) — no locks, no I/O, no complex computation
  std::unordered_map<ReadCommandsNC::ReadCommand,
                     std::function<void(const uint8_t *, size_t)>>
      read_callbacks{};
  std::unordered_map<WriteCommandsNC::WriteCommand,
                     std::function<void(const uint8_t *, size_t)>>
      write_callbacks{};

  explicit Protocol_Handler_I2C(const std::string &device_in = "",
                                int slave_addr_in = 0x00);

  /// Register optional log callbacks. Call before init() if you want
  /// construction-time messages. Each level can be set independently;
  /// unset levels are silent.
  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    log_info_  = std::move(info);
    log_warn_  = std::move(warn);
    log_error_ = std::move(error);
  }

  /// Read-only access to the last error state.
  Error_State getErrorState() const { return error_state; }

  /// Clear a transient IO_ERROR without reconnecting.
  /// IO_ERROR is the only error that leaves the fd open — all others
  /// either never opened it or already closed it.
  void clearError() {
    if (error_state == Error_State::IO_ERROR)
      error_state = Error_State::NONE;
  }

  /// Initialise an already-constructed (default) handler and attempt connection.
  bool init(const std::string &device_in, int slave_addr_in);

  /// Close any existing fd and re-attempt connection. Returns true on success.
  bool reconnect();

  ~Protocol_Handler_I2C() noexcept;

  bool connect();

  bool addCommand(std::unique_ptr<CommandsNC::Command> &&input);

  // TODO: add sequencing to the protocol
  bool sendQueue(micros timeout = micros(8000), micros timePerMsg = micros(4000));

  /// Read pending messages until there are none left.
  /// Returns true if at least one message was dispatched, false on I/O error
  /// or if no messages were available.
  bool readPending(micros timeout = micros(8000), micros timePerMsg = micros(4000));

  bool connected() const;
  const std::string &getDevice() const;
  int getSlaveAddress() const;

private:
  enum class ReadResult {
    OK_DISPATCHED,
    NO_MESSAGE,
    NO_SYNC,
    CRC_MISMATCH,
    ERROR_IO, 
  };

  size_t     sendNext(deadline_t deadline);
  /// Assumes sync byte (0xAA) has already been found and stored in synced_byte_.
  ReadResult readOneMessage(micros timePerMsg);
  uint8_t    getMsgCRC(const header &msg_head, uint8_t *pckage, size_t size);
  /// Reads the rest of the header after the sync byte has already been found.
  ReadResult ReadHeader(header &output, deadline_t hdr_dl);
  /// Wait until i2c_fd is ready for I/O or the deadline expires.
  /// Uses ppoll() for nanosecond-resolution timeout.
  bool       waitFdReady(short events, deadline_t deadline);

  // Low-level I/O — all I2C hardcoding lives here; do not modify unless
  // the underlying protocol framing changes.
  size_t writeData(const uint8_t *data, size_t length, deadline_t deadline);
  size_t readData(uint8_t *buffer, size_t length, deadline_t deadline);

  void closeI2C();
  bool isValidConfig() const noexcept;
  void dispatchInput(uint8_t inst, uint8_t *pckg, size_t size);

  // Internal buffer constants.
  static constexpr size_t INTERNAL_BUF_SIZE  = 8;

  static constexpr size_t MAX_PAYLOAD_SIZE = 60;

  static_assert(INTERNAL_BUF_SIZE != 3,
      "INTERNAL_BUF_SIZE == 3 collides with the no-message sentinel frame");

  uint8_t internal_buf[INTERNAL_BUF_SIZE];
  // Start "full" so the first readData call immediately fetches a chunk.
  size_t internal_pos = INTERNAL_BUF_SIZE;

  // Sync state — readPending finds 0xAA and stores it here so readOneMessage
  // can consume it without re-scanning. Cleared after each message attempt.
  bool    sync_pending_ { false };
  uint8_t synced_byte_  { 0x00 };

  constexpr static unsigned int max_queue{50};

  std::string device;
  int i2c_fd{-1}, slave_addr{-1};
  Error_State error_state{Error_State::CLOSED};

  std::queue<std::unique_ptr<CommandsNC::Command>> sending;

  std::unordered_map<uint8_t, std::function<void(const uint8_t *, size_t)>>
      instruction_callback{};

  // Log callbacks — all silent by default.
  LogFn log_info_{};
  LogFn log_warn_{};
  LogFn log_error_{};

  // Internal log helpers — format once and dispatch.
  template<typename... Args>
  void logInfo (const char *fmt, Args&&... args) const { logDispatch(log_info_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logWarn (const char *fmt, Args&&... args) const { logDispatch(log_warn_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logError(const char *fmt, Args&&... args) const { logDispatch(log_error_, fmt, std::forward<Args>(args)...); }

  template<typename... Args>
  static void logDispatch(const LogFn &fn, const char *fmt, Args&&... args) {
    if (!fn) return;
    // Use snprintf into a fixed buffer — avoids heap allocation on hot paths.
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, std::forward<Args>(args)...);
    fn(buf);
  }
};

// =============================================================================
// Inline definitions
// =============================================================================

inline std::string bytes_to_hex_string(const uint8_t* buf, size_t size)
{
    std::ostringstream ss;

    for (size_t i = 0; i < size; i++) {
        ss << "0x"
           << std::hex
           << std::setw(2)
           << std::setfill('0')
           << static_cast<int>(buf[i])
           << " ";
    }

    return ss.str();
}

inline Protocol_Handler_I2C::Protocol_Handler_I2C(const std::string &device_in,
                                                   int slave_addr_in)
    : device{device_in}, slave_addr{slave_addr_in} {
  if (!device.empty()) {
    connect();
  }

  // Instruction 1 = Ping / static commands (no payload dispatch needed)
  instruction_callback.emplace(1, [](const uint8_t *pckg, size_t size) {
    (void)pckg;
    (void)size;
  });

  // Instruction 2 = Write command responses
  instruction_callback.emplace(2, [this](const uint8_t *pckg, size_t size) {
    uint8_t id = pckg[0];
    auto it = write_callbacks.find(
        static_cast<WriteCommandsNC::WriteCommand>(id));
    if (it != write_callbacks.end()) {
      it->second(pckg + 1, size - 1);
    }
  });

  // Instruction 3 = Read command responses
  instruction_callback.emplace(3, [this](const uint8_t *pckg, size_t size) {
    uint8_t id = pckg[0];
    auto it = read_callbacks.find(static_cast<ReadCommandsNC::ReadCommand>(id));
    if (it != read_callbacks.end()) {
      it->second(pckg + 1, size - 1);
    }
  });
}

inline bool Protocol_Handler_I2C::init(const std::string &device_in,
                                        int slave_addr_in) {
  device     = device_in;
  slave_addr = slave_addr_in;
  return connect();
}

inline bool Protocol_Handler_I2C::reconnect() {
  if (device.empty()) {
    error_state = Error_State::INVALID_CONFIG;
    return false;
  }
  return connect();
}

inline Protocol_Handler_I2C::~Protocol_Handler_I2C() noexcept { closeI2C(); }

inline bool Protocol_Handler_I2C::connect() {
  closeI2C();
  error_state = Error_State::NONE;
  if (!isValidConfig()) {
    error_state = Error_State::INVALID_CONFIG;
    return false;
  }
  i2c_fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (i2c_fd < 0) {
    error_state = Error_State::OPEN_FAILED;
    return false;
  }
  
  if (flock(i2c_fd, LOCK_EX | LOCK_NB) != 0) {
    if (errno == EWOULDBLOCK) {
        error_state = Error_State::DEVICE_BUSY;
        logError("Device is already in use");
    } else {
        error_state = Error_State::LOCK_FAILED;
        logError("Failed to lock file");
    }
    closeI2C();
    return false;
}

  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    error_state = Error_State::IOCTL_FAILED;
    closeI2C();
    return false;
  }
  logInfo("Managed to connect %s at %d", device.c_str(),
              slave_addr);
  return true;
}

inline bool Protocol_Handler_I2C::addCommand(
    std::unique_ptr<CommandsNC::Command> &&input) {
  if (sending.size() < max_queue) {
    sending.push(std::move(input));
    return true;
  }
  return false;
}

inline bool Protocol_Handler_I2C::sendQueue(micros timeout, micros timePerMsg) {
  if (!connected())
    return false;

  const deadline_t dl = stdclock::now() + timeout;
  size_t bytes{0};

  // Per-tick send budget: flush at most one full I2C transaction worth of
  // data per sendQueue() call — MAX_PAYLOAD_SIZE + header(3) + tail(1).
  // Prevents one large burst from consuming the entire tick budget.
  constexpr size_t SEND_BUDGET =
      MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{});

  size_t expectedsize{0};
  while (!sending.empty() &&
         (expectedsize = sending.front()->getPckSize() +
                         tuple_size(header{}) + tuple_size(tail{})) &&
         bytes + expectedsize <= SEND_BUDGET && stdclock::now() + timePerMsg < dl) {
    auto sent = sendNext(dl);
    if (sent != expectedsize) {
      if (error_state == Error_State::NONE)
        error_state = Error_State::IO_ERROR;
      return false;
    }
    bytes += sent;
  }
  return true;
}

inline size_t Protocol_Handler_I2C::sendNext(deadline_t deadline) {
  if (!connected() || sending.empty())
    return 0;

  auto size = sending.front()->getPckSize();
  constexpr auto headerSize = tuple_size(header{});
  constexpr auto tailSize   = tuple_size(tail{});

  // Stack-allocated frame: MAX_PAYLOAD_SIZE + header(3) + tail(1) = 64 bytes max.
  std::array<uint8_t, MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{})> buffer{};
  const size_t frameSize = size + headerSize + tailSize;

  auto headerSend =
      make_msg_from_args(header{}, 0xAA, sending.front()->getInst(), size);
  pack_tuple_to_buffer(headerSend, buffer.data());
  sending.front()->pack(buffer.data() + headerSize);

  auto tail_tuple = make_msg_from_args(
      tail{}, calcCRC(buffer.data(), headerSize + size, 0));
  pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + size);

  auto sent = writeData(buffer.data(), frameSize, deadline);
  if (sent == frameSize) {
    sending.pop();
  }
  return sent;
}

inline size_t Protocol_Handler_I2C::writeData(const uint8_t *data,
                                               size_t length, deadline_t deadline) {
  if (!connected() || length == 0)
    return 0;

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

  // Single ioctl — no chunking, no padding. The frame is already fully
  // assembled by sendNext and fits within MAX_PAYLOAD_SIZE + framing = 64 bytes,
  // well within the kernel i2c-rdwr limit.
  struct i2c_msg msg{};
  msg.addr  = static_cast<__u16>(slave_addr);
  msg.flags = 0;
  msg.len   = static_cast<__u16>(length);
  msg.buf   = const_cast<uint8_t *>(data);

  struct i2c_rdwr_ioctl_data ioctl_data{};
  ioctl_data.msgs  = &msg;
  ioctl_data.nmsgs = 1;

  int ret = ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
  if (ret != static_cast<int>(ioctl_data.nmsgs)) {
    closeI2C();
    error_state = Error_State::IOCTL_FAILED;
    return 0;
  }

  return length;
}

inline bool Protocol_Handler_I2C::readPending(micros timeout, micros timePerMsg) {
  if (!connected())
    return false;

  const deadline_t dl = stdclock::now() + timeout;
  bool dispatched_any = false;

  while (true) {
    // Find sync byte using the full remaining deadline — this is not charged
    // against timePerMsg. Only enter the message path if budget still allows it.
    if (!sync_pending_) {
      // Scan for 0xAA sync byte — consumes the global deadline, not timePerMsg.
      bool found = false;
      uint8_t byte = 0;
      while (stdclock::now() < dl) {
        if (readData(&byte, 1, dl) != 1) break;
        if (byte == 0xAA) { found = true; break; }
      }
      if (!found) {
        logWarn("Could not sync to message HEADER");
        break;
      }
      synced_byte_  = byte;
      sync_pending_ = true;
    }

    // Sync found — check budget before committing to reading a full message.
    if (stdclock::now() + timePerMsg >= dl) break;

    auto res = readOneMessage(timePerMsg);
    if (res == ReadResult::OK_DISPATCHED) {
      //logInfo("Message dispatched successfully");
      dispatched_any = true;
      continue;
    }
    if (res == ReadResult::NO_MESSAGE) {
      //logInfo("No message available to read");
      break;
    }
    if (res == ReadResult::CRC_MISMATCH) {
      //logWarn("CRC mismatch for received message, dropping it");
      continue;
    }
    logError("I/O error occurred while reading message");
    if (error_state == Error_State::NONE)
      error_state = Error_State::IO_ERROR;
    return false;
  }

  return dispatched_any;
}

inline Protocol_Handler_I2C::ReadResult Protocol_Handler_I2C::readOneMessage(micros timePerMsg) {
  if (!connected()) {
    sync_pending_ = false;
    return ReadResult::ERROR_IO;
  }

  // Per-message deadline starts now — sync was already found by readPending.
  const deadline_t msg_dl = stdclock::now() + timePerMsg;

  header msg_head;
  auto state = ReadHeader(msg_head, msg_dl);
  if (state != ReadResult::OK_DISPATCHED) {
    return state;
  }

  const uint8_t inst   = std::get<1>(msg_head);
  const uint8_t length = std::get<2>(msg_head);

  // Guard against malformed length before touching the stack buffer.
  if (length > MAX_PAYLOAD_SIZE) {
    logWarn("Payload length %u exceeds MAX_PAYLOAD_SIZE (%zu) — dropping",
            static_cast<unsigned>(length), MAX_PAYLOAD_SIZE);
    return ReadResult::ERROR_IO;
  }
  // FIX: always read length+1 bytes so the CRC byte is always fetched,
  // even when length == 0 (original skipped readData for length==0,
  // leaving out_buffer[0] uninitialised).
  // Stack-allocated: MAX_PAYLOAD_SIZE + 1 (CRC) = 61 bytes worst case.
  std::array<uint8_t, MAX_PAYLOAD_SIZE + 1> out_buffer{};
  auto num = readData(out_buffer.data(), length + 1, msg_dl);
  if (num != static_cast<size_t>(length + 1)) {
    return ReadResult::ERROR_IO;
  }

  uint8_t recv_crc       = out_buffer[length];
  auto    calculated_crc = getMsgCRC(msg_head, out_buffer.data(), length);

  //logInfo("Received and Calculated CRCs: 0x%02X, 0x%02X", recv_crc, calculated_crc);

  if (recv_crc != calculated_crc) {
    //logWarn("CRC mismatch: Inst, Size, ReadCRC n CalcCRC: 0x%02X, %03i, 0x%02X, 0x%02X", inst, length, recv_crc, calculated_crc);
    return ReadResult::CRC_MISMATCH;
  }

  /*logInfo("Received instruction 0x%02X with payload size %d",
               inst, length);*/
  dispatchInput(inst, out_buffer.data(), length);
  return ReadResult::OK_DISPATCHED;
}

inline Protocol_Handler_I2C::ReadResult
Protocol_Handler_I2C::ReadHeader(header &output, deadline_t hdr_dl) {

  uint8_t header_buf[sizeof(output)];
  header_buf[0] = synced_byte_;   // 0xAA — already validated by readPending
  sync_pending_ = false;           // consumed

  auto num = readData(header_buf + 1, sizeof(header_buf) - 1, hdr_dl);
  if (num != (sizeof(header_buf) - 1)) {
    return ReadResult::ERROR_IO;
  }
  unpack_tuple_from_buffer(output, header_buf);

  // Sentinel check: if the MCU had nothing to send it emits 0xAA 0x00 CRC.
  // readPending will re-sync on the next iteration.
  if (output == header{0xAA, 0x00, calcCRC({0xAA, 0x00}, 0)})
    return ReadResult::NO_MESSAGE;

  return ReadResult::OK_DISPATCHED;
}

inline uint8_t Protocol_Handler_I2C::getMsgCRC(const header &msg_head,
                                                uint8_t *pckage, size_t size) {
  uint8_t tmp[sizeof(header)];
  pack_tuple_to_buffer(msg_head, tmp);
  uint8_t crc = calcCRC(tmp, sizeof(header), 0);
  crc         = calcCRC(pckage, size, crc);
  return crc;
}

inline size_t Protocol_Handler_I2C::readData(uint8_t *buffer, size_t length,
                                              deadline_t deadline) {
  if (!connected() || !buffer || length == 0)
    return 0;

  size_t total = 0;

  if (internal_pos < INTERNAL_BUF_SIZE) {
    size_t cached   = INTERNAL_BUF_SIZE - internal_pos;
    size_t to_copy  = std::min(cached, length);
    std::memcpy(buffer, internal_buf + internal_pos, to_copy);
    internal_pos += to_copy;
    total        += to_copy;
  }

  if (total == length)
    return total;  

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

    // Read ioctl
    struct i2c_msg msgs[2]{};
    msgs[0].addr  = static_cast<__u16>(slave_addr);
    msgs[0].flags = I2C_M_RD;
    msgs[0].len   = static_cast<__u16>(fetchSize);
    msgs[0].buf   = dest;

    // Skip: tell the MCU how many bytes were consumed so it advances its TX pointer.
    // This should have been a repeated start but some i2c drivers do not support
    // read-then-write in a single I2C_RDWR ioctl, so we use two separate ioctls.
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
        closeI2C();
        error_state = Error_State::IOCTL_FAILED;
        return false;
      }
    }
    return true;
  };

  if (remaining <= INTERNAL_BUF_SIZE) {
    // Small fetch — refill internal cache, copy what is needed, keep the rest.
    if (!doFetch(internal_buf, INTERNAL_BUF_SIZE))
      return total;
    internal_pos = 0;
    std::memcpy(buffer + total, internal_buf, remaining);
    internal_pos = remaining;
    total        = length;
  } else {
    // Large fetch — read directly into the caller's buffer in one syscall.
    if (!doFetch(buffer + total, remaining))
      return total;

    total = length;
  }

  return total;
}

inline bool Protocol_Handler_I2C::waitFdReady(short events, deadline_t deadline) {
  struct pollfd pfd{};
  pfd.fd     = i2c_fd;
  pfd.events = events;

  // Convert absolute deadline to a relative timespec for ppoll().
  // ppoll() gives nanosecond-resolution timeout unlike poll()'s milliseconds,
  // which matters when the remaining budget is in the low-microsecond range.
  auto remaining_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline - stdclock::now());
  if (remaining_ns.count() <= 0) {
    return false;   // already expired — don't even call ppoll
  }

  struct timespec ts{};
  ts.tv_sec  = remaining_ns.count() / 1'000'000'000LL;
  ts.tv_nsec = remaining_ns.count() % 1'000'000'000LL;

  int ret = ppoll(&pfd, 1, &ts, nullptr);
  if (ret < 0) {
    if (errno == EBADF) {
      // fd is invalid according to the kernel — close and mark FD_INVALID.
      // closeI2C() sets CLOSED, so we override with FD_INVALID afterwards
      // to give the caller a more specific reason.
      closeI2C();
      error_state = Error_State::FD_INVALID;
    }
    // EINTR and other errors are not fatal to the fd — caller sees false
    // and handles via its own deadline logic.
    return false;
  }
  
  return ret > 0 && (pfd.revents & events);
}

inline void Protocol_Handler_I2C::closeI2C() {
  if (i2c_fd >= 0) {
    ::close(i2c_fd);
    i2c_fd = -1;

    std::queue<std::unique_ptr<CommandsNC::Command>> empty;
    std::swap(sending, empty);

    logInfo("Closed connection to %s at %d", device.c_str(),
                slave_addr);
    
    if(error_state == Error_State::NONE || error_state == Error_State::IO_ERROR)
      error_state = Error_State::CLOSED;
  }
}

inline bool Protocol_Handler_I2C::connected() const { return i2c_fd >= 0; }

inline const std::string &Protocol_Handler_I2C::getDevice() const {
  return device;
}

inline int Protocol_Handler_I2C::getSlaveAddress() const { return slave_addr; }

inline bool Protocol_Handler_I2C::isValidConfig() const noexcept {
  return !device.empty() && slave_addr >= 0x03 && slave_addr <= 0x77;
}

inline void Protocol_Handler_I2C::dispatchInput(uint8_t inst, uint8_t *pckg,
                                                 size_t size) {
  // TODO: register timestamp of incoming messages
  auto it = instruction_callback.find(inst);
  if (it != instruction_callback.end()) {
    it->second(pckg, size);
  }
}