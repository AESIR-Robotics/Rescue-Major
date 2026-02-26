#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "commands.hpp"
#include "crc.hpp"
#include "tuple_utils.hpp"

using micros   = std::chrono::microseconds;
using stdclock = std::chrono::steady_clock;

// =============================================================================
// Declaration
// =============================================================================

class Protocol_Handler_I2C {
public:
  enum class Error_State {
    NONE,
    INVALID_CONFIG,
    OPEN_FAILED,
    IOCTL_FAILED,
    IO_ERROR
  };

  using header = std::tuple<uint8_t, uint8_t, uint8_t>; // sync, instruction, length
  using tail   = std::tuple<uint8_t>;                    // checksum

  std::unordered_map<ReadCommandsNC::ReadCommand,
                     std::function<void(const uint8_t *, size_t)>>
      read_callbacks{};
  std::unordered_map<WriteCommandsNC::WriteCommand,
                     std::function<void(const uint8_t *, size_t)>>
      write_callbacks{};

  explicit Protocol_Handler_I2C(rclcpp::Logger log,
                                const std::string &device_in = "",
                                int slave_addr_in = 0x00);

  /// Initialise an already-constructed (default) handler and attempt connection.
  bool init(const std::string &device_in, int slave_addr_in);

  /// Close any existing fd and re-attempt connection. Returns true on success.
  bool reconnect();

  ~Protocol_Handler_I2C() noexcept;

  bool connect();

  bool addCommand(std::unique_ptr<CommandsNC::Command> &&input);

  // TODO: add sequencing to the protocol
  bool sendQueue(micros timeout = micros(5000));

  /// Read pending messages until there are none left.
  /// Returns true if at least one message was dispatched, false on I/O error
  /// or if no messages were available.
  bool readPending(micros timeout = micros(5000));

  bool connected() const;
  const std::string &getDevice() const;
  int getSlaveAddress() const;

private:
  enum class ReadResult {
    OK_DISPATCHED,
    NO_MESSAGE,
    NO_SYNC,
    CRC_MISMATCH,
    ERROR_IO
  };

  size_t     sendNext();
  ReadResult readOneMessage();
  uint8_t    getMsgCRC(const header &msg_head, uint8_t *pckage, size_t size);
  ReadResult ReadHeader(header &output);
  bool       syncMessage(uint8_t &first, micros time = micros(3000));

  // Low-level I/O — all I2C hardcoding lives here; do not modify unless
  // the underlying protocol framing changes.
  size_t writeData(const uint8_t *data, size_t length,
                   micros timeout = micros{1500});
  size_t readData(uint8_t *buffer, size_t length,
                  micros timeout = micros{2000});

  void closeI2C();
  bool isValidConfig() const noexcept;
  void dispatchInput(uint8_t inst, uint8_t *pckg, size_t size);

  // Internal buffer constants.
  // WARNING: do not set INTERNAL_SEND_SIZE to 4 — causes obscure framing bugs.
  static constexpr size_t INTERNAL_SEND_SIZE = 16;
  static constexpr size_t INTERNAL_BUF_SIZE  = 8;

  uint8_t internal_buf[INTERNAL_BUF_SIZE];
  // Start "full" so the first readData call immediately fetches a chunk.
  size_t internal_pos = INTERNAL_BUF_SIZE;

  constexpr static unsigned int max_queue{50};

  std::string device;
  int i2c_fd{-1}, slave_addr{-1};
  Error_State error_state{Error_State::OPEN_FAILED};

  std::queue<std::unique_ptr<CommandsNC::Command>> sending;

  std::unordered_map<uint8_t, std::function<void(const uint8_t *, size_t)>>
      instruction_callback{};

  rclcpp::Logger logger;
};

// =============================================================================
// Templates
// =============================================================================

// (none — Protocol_Handler_I2C is not a template class)

// =============================================================================
// Inline definitions
// =============================================================================

inline Protocol_Handler_I2C::Protocol_Handler_I2C(rclcpp::Logger log,
                                                   const std::string &device_in,
                                                   int slave_addr_in)
    : device{device_in}, slave_addr{slave_addr_in}, logger{log} {
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
  i2c_fd = open(device.c_str(), O_RDWR);
  if (i2c_fd < 0) {
    error_state = Error_State::OPEN_FAILED;
    return false;
  }
  if (ioctl(i2c_fd, I2C_SLAVE, slave_addr) < 0) {
    error_state = Error_State::IOCTL_FAILED;
    closeI2C();
    return false;
  }
  RCLCPP_INFO(logger, "Managed to connect %s at %d", device.c_str(),
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

inline bool Protocol_Handler_I2C::sendQueue(micros timeout) {
  if (!connected())
    return false;

  size_t bytes{0};
  auto   deadline = stdclock::now() + timeout;

  size_t expectedsize{0};
  while (!sending.empty() &&
         (expectedsize = sending.front()->getPckSize() +
                         tuple_size(header{}) + tuple_size(tail{})) &&
         bytes + expectedsize < 100 && stdclock::now() < deadline) {
    auto sent = sendNext();
    if (sent != expectedsize) {
      return false;
    }
    bytes += sent;
  }

  return true;
}

inline bool Protocol_Handler_I2C::readPending(micros timeout) {
  if (!connected())
    return false;

  bool dispatched_any = false;
  for (auto deadline = stdclock::now();
       stdclock::now() < deadline + timeout;) {
    auto res = readOneMessage();
    if (res == ReadResult::OK_DISPATCHED) {
      RCLCPP_DEBUG(logger, "Message dispatched successfully");
      dispatched_any = true;
      continue;
    }
    if (res == ReadResult::NO_MESSAGE) {
      RCLCPP_DEBUG(logger, "No message available to read");
      break;
    }
    if (res == ReadResult::NO_SYNC) {
      RCLCPP_WARN(logger, "Could not sync to message HEADER");
      break;
    }
    if (res == ReadResult::CRC_MISMATCH) {
      RCLCPP_WARN(logger, "CRC mismatch for received message, dropping it");
      continue;
    }
    RCLCPP_ERROR(logger, "I/O error occurred while reading message");
    return false;
  }

  return dispatched_any;
}

inline bool Protocol_Handler_I2C::connected() const { return i2c_fd > 0; }

inline const std::string &Protocol_Handler_I2C::getDevice() const {
  return device;
}

inline int Protocol_Handler_I2C::getSlaveAddress() const { return slave_addr; }

inline size_t Protocol_Handler_I2C::sendNext() {
  if (!connected() || sending.empty())
    return 0;

  auto size = sending.front()->getPckSize();
  constexpr auto headerSize = tuple_size(header{});
  constexpr auto tailSize   = tuple_size(tail{});
  std::vector<uint8_t> buffer(size + headerSize + tailSize);

  auto headerSend =
      make_msg_from_args(header{}, 0xAA, sending.front()->getInst(), size);
  pack_tuple_to_buffer(headerSend, buffer.data());
  sending.front()->pack(buffer.data() + headerSize);

  auto tail_tuple = make_msg_from_args(
      tail{}, calcCRC(buffer.data(), headerSize + size, 0));
  pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + size);

  auto sent = writeData(buffer.data(), buffer.size());
  if (sent == buffer.size()) {
    sending.pop();
  }
  return sent;
}

inline Protocol_Handler_I2C::ReadResult Protocol_Handler_I2C::readOneMessage() {
  if (!connected()) {
    RCLCPP_DEBUG(logger, "Not connected");
    return ReadResult::ERROR_IO;
  }

  header msg_head;
  auto state = ReadHeader(msg_head);
  if (state != ReadResult::OK_DISPATCHED) {
    return state;
  }

  uint8_t inst   = std::get<1>(msg_head);
  uint8_t length = std::get<2>(msg_head);

  // FIX: always read length+1 bytes so the CRC byte is always fetched,
  // even when length == 0 (original skipped readData for length==0,
  // leaving out_buffer[0] uninitialised).
  std::vector<uint8_t> out_buffer(length + 1);
  {
    auto num = readData(out_buffer.data(), length + 1);
    if (num != static_cast<size_t>(length + 1)) {
      error_state = Error_State::IO_ERROR;
      closeI2C();
      return ReadResult::ERROR_IO;
    }
    if (length > 0) {
      RCLCPP_DEBUG(logger,
                   "Payload read for instruction 0x%02X: first 4 bytes "
                   "0x%02X 0x%02X 0x%02X 0x%02X",
                   inst, out_buffer[0],
                   out_buffer.size() > 1 ? out_buffer[1] : 0,
                   out_buffer.size() > 2 ? out_buffer[2] : 0,
                   out_buffer.size() > 3 ? out_buffer[3] : 0);
    }
  }

  uint8_t recv_crc       = out_buffer[length];
  auto    calculated_crc = getMsgCRC(msg_head, out_buffer.data(), length);

  RCLCPP_DEBUG(logger, "Received CRC: 0x%02X", recv_crc);

  if (recv_crc != calculated_crc) {
    return ReadResult::CRC_MISMATCH;
  }

  RCLCPP_DEBUG(logger, "Received instruction 0x%02X with payload size %d",
               inst, length);
  dispatchInput(inst, out_buffer.data(), length);
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

inline Protocol_Handler_I2C::ReadResult
Protocol_Handler_I2C::ReadHeader(header &output) {
  uint8_t first = 0x00;
  if (!syncMessage(first)) {
    return ReadResult::NO_SYNC;
  }

  if (first != 0xAA) {
    return ReadResult::NO_SYNC;
  }

  uint8_t header_buf[sizeof(output)];
  header_buf[0] = first;
  auto num = readData(header_buf + 1, sizeof(header_buf) - 1);
  if (num != (sizeof(header_buf) - 1)) {
    error_state = Error_State::IO_ERROR;
    closeI2C();
    return ReadResult::ERROR_IO;
  }

  unpack_tuple_from_buffer(output, header_buf);

  if (output == header{0xAA, 0x00, calcCRC({0xAA, 0x00}, 0)}) {
    // "No message" sentinel — if it started in the last chunk there might
    // still be a real message right after it.
    if (internal_pos < tuple_size(header{})) {
      return ReadHeader(output);
    }
    return ReadResult::NO_MESSAGE;
  }

  return ReadResult::OK_DISPATCHED;
}

inline bool Protocol_Handler_I2C::syncMessage(uint8_t &first, micros time) {
  auto deadline = stdclock::now() + time;
  uint8_t byte;
  while (stdclock::now() < deadline) {
    if (readData(&byte, 1, micros(1000)) != 1) {
      return false;
    }
    first = byte;
    if (byte == 0xAA) {
      return true;
    }
  }
  RCLCPP_INFO(logger, "Sync byte not found within timeout");
  return false;
}

inline size_t Protocol_Handler_I2C::writeData(const uint8_t *data,
                                               size_t length, micros timeout) {
  if (!connected())
    return 0;

  auto   deadline = stdclock::now() + timeout;
  size_t total{0};

  RCLCPP_DEBUG(logger, "Writing a message with %zu bytes", length);
  while (total < length) {
    size_t available = std::min(INTERNAL_SEND_SIZE, length - total);

    std::array<uint8_t, INTERNAL_SEND_SIZE> buf;
    memcpy(buf.data(), data + total, available);
    for (auto i = available; i < INTERNAL_SEND_SIZE; i++) {
      buf[i] = 0xBB;
    }

    struct i2c_msg msg {};
    msg.addr  = static_cast<__u16>(slave_addr);
    msg.flags = 0;
    msg.len   = static_cast<__u16>(INTERNAL_SEND_SIZE);
    msg.buf   = buf.data();

    struct i2c_rdwr_ioctl_data ioctl_data {};
    ioctl_data.msgs  = &msg;
    ioctl_data.nmsgs = 1;

    int ret = ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
    if (ret != static_cast<int>(ioctl_data.nmsgs)) {
      closeI2C();
      error_state = Error_State::IO_ERROR;
      return total;
    }

    total += available;

    if (stdclock::now() > deadline) {
      RCLCPP_WARN(logger,
                  "Send timeout from device %s at address %d: expected %zu "
                  "bytes, got %zu",
                  device.c_str(), slave_addr, length, total);
      break;
    }
  }

  return total;
}

inline size_t Protocol_Handler_I2C::readData(uint8_t *buffer, size_t length,
                                              micros timeout) {
  if (!connected() || !buffer || length == 0)
    return 0;

  auto   deadline = stdclock::now() + timeout;
  size_t total    = 0;

  while (total < length) {
    // Refill internal buffer when exhausted
    if (internal_pos >= INTERNAL_BUF_SIZE) {
      RCLCPP_DEBUG(logger, "Reading new chunk...");

      // Send "skip" message to advance microcontroller's TX pointer
      uint8_t skipmsg[4]{0xBB, 0xFF, INTERNAL_BUF_SIZE, 0x00};
      skipmsg[3] = calcCRC(skipmsg, 3, 0);

      struct i2c_msg msg[2]{};
      msg[0].addr  = static_cast<__u16>(slave_addr);
      msg[0].flags = I2C_M_RD;
      msg[0].len   = INTERNAL_BUF_SIZE;
      msg[0].buf   = internal_buf;

      msg[1].addr  = static_cast<__u16>(slave_addr);
      msg[1].flags = 0;
      msg[1].len   = 4;
      msg[1].buf   = skipmsg;

      struct i2c_rdwr_ioctl_data ioctl_data {};
      ioctl_data.msgs  = msg;
      ioctl_data.nmsgs = 2;

      int ret = ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
      if (ret != static_cast<int>(ioctl_data.nmsgs)) {
        closeI2C();
        error_state = Error_State::IO_ERROR;
        return total;
      }

      internal_pos = 0;
    }

    size_t available = INTERNAL_BUF_SIZE - internal_pos;
    size_t to_copy   = std::min(available, length - total);

    std::memcpy(buffer + total, internal_buf + internal_pos, to_copy);
    internal_pos += to_copy;
    total        += to_copy;

    if (stdclock::now() > deadline) {
      RCLCPP_WARN(logger,
                  "Read timeout from device %s at address %d: expected %zu "
                  "bytes, got %zu",
                  device.c_str(), slave_addr, length, total);
      break;
    }
  }

  return total;
}

inline void Protocol_Handler_I2C::closeI2C() {
  if (i2c_fd >= 0) {
    ::close(i2c_fd);
    i2c_fd = -1;

    std::queue<std::unique_ptr<CommandsNC::Command>> empty;
    std::swap(sending, empty);

    RCLCPP_INFO(logger, "Closed connection to %s at %d", device.c_str(),
                slave_addr);

    error_state = Error_State::OPEN_FAILED;
  }
}

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