#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <cstring>
#include <initializer_list>
#include <iterator>
#include <memory>

#include <mutex>
#include <queue>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <limits>

#include <math.h>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <fcntl.h> // open
#include <sys/types.h>
#include <termios.h> // termios
#include <tuple>
#include <unistd.h> // write, close

#include <type_traits>
#include <unordered_map>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
// #include "control_msgs/msg/joint_jog.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "hardware/msg/joint_control.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Compute total byte size of a tuple (sum of sizeof each sending.front()ent
// type)
template <typename... Args>
constexpr size_t tuple_size(const std::tuple<Args...> &) {
  return (sizeof(Args) + ... + 0);
}

// Helpers to pack/unpack a tuple into a contiguous byte buffer
// Recursive tuple pack/unpack helpers (safer and easier to reason about)
template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
pack_tuple_recursive(const std::tuple<Args...> &, uint8_t * /*buffer*/,
                     size_t & /*offset*/) {
  // end recursion
}

template <size_t I = 0, typename... Args>
    std::enable_if_t < I<sizeof...(Args), void>
                       pack_tuple_recursive(const std::tuple<Args...> &tup,
                                            uint8_t *buffer, size_t &offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  std::memcpy(buffer + offset, &std::get<I>(tup), sizeof(T));
  offset += sizeof(T);
  pack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void pack_tuple_to_buffer(const std::tuple<Args...> &tuple, uint8_t *buffer) {
  size_t offset = 0;
  pack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
unpack_tuple_recursive(std::tuple<Args...> &, const uint8_t * /*buffer*/,
                       size_t & /*offset*/) {
  // end recursion
}

template <size_t I = 0, typename... Args>
    std::enable_if_t <
    I<sizeof...(Args), void> unpack_tuple_recursive(std::tuple<Args...> &tup,
                                                    const uint8_t *buffer,
                                                    size_t &offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  std::memcpy(&std::get<I>(tup), buffer + offset, sizeof(T));
  offset += sizeof(T);
  unpack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void unpack_tuple_from_buffer(std::tuple<Args...> &tuple,
                              const uint8_t *buffer) {
  size_t offset = 0;
  unpack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

template <typename Tuple, typename... Args> struct args_match_tuple;

template <typename... Ts, typename... Args>
struct args_match_tuple<std::tuple<Ts...>, Args...> {
  static constexpr bool value = (sizeof...(Ts) == sizeof...(Args)) &&
                                (std::is_constructible_v<Ts, Args> && ...);
};

template <typename... TupleArgs, typename... Args>
auto make_msg_from_args(std::tuple<TupleArgs...>, const Args &...args) {
  static_assert(args_match_tuple<std::tuple<TupleArgs...>, Args...>::value,
                "Argument types do not match tuple types");

  // Static cast of every compatible argument to create std::tuple<TupleArgs...>
  return std::tuple<TupleArgs...>{(static_cast<TupleArgs>(args))...};
}

template <typename Tuple, typename NewType> struct tuple_push_back;

template <typename... Ts, typename NewType>
struct tuple_push_back<std::tuple<Ts...>, NewType> {
  using type = std::tuple<Ts..., NewType>;
};

template <typename Tuple, typename NewType>
using tuple_push_back_t = typename tuple_push_back<Tuple, NewType>::type;

template <typename Tuple, typename NewType> struct tuple_push_front;

template <typename... Ts, typename NewType>
struct tuple_push_front<std::tuple<Ts...>, NewType> {
  using type = std::tuple<NewType, Ts...>;
};

template <typename Tuple, typename NewType>
using tuple_push_front_t = typename tuple_push_front<Tuple, NewType>::type;

namespace WriteCommandsNC {

enum WriteCommand : uint8_t { DIRECTION = 0x00, SPEED = 0x01, POSITION = 0x02 };

template <WriteCommand CMD> struct packetSend {
  using type = void;
};

template <WriteCommand CMD> struct packetReturn {
  using type = void;
};

template <> struct packetSend<DIRECTION> {
  using type = std::tuple<uint8_t, bool>; // motor Mask, direction
};

template <> struct packetReturn<DIRECTION> {
  using type = std::tuple<>;
};

template <> struct packetSend<SPEED> {
  using type = std::tuple<uint8_t, float>; // motor Mask, speed
};

template <> struct packetReturn<SPEED> {
  using type = std::tuple<>;
};

template <> struct packetSend<POSITION> {
  using type = std::tuple<uint8_t, int32_t>; // motor Mask, position
};

template <> struct packetReturn<POSITION> {
  using type = std::tuple<>;
};

} // namespace WriteCommandsNC

namespace ReadCommandsNC {

// constexpr static u_int16_t a = 0x01 | (0x01 << 8);
enum ReadCommand : uint8_t { DIRECTION = 0x00, SPEED = 0x01, POSITION = 0x02 };

template <ReadCommand CMD> struct packetSend {
  using type = void;
};

template <ReadCommand CMD> struct packetReturn {
  using type = void;
};

template <> struct packetSend<DIRECTION> {
  using type = std::tuple<>;
};

template <> struct packetReturn<DIRECTION> {
  using type = std::tuple<bool, bool, bool, bool>; // motor Mask, direction
};

template <> struct packetSend<SPEED> {
  using type = std::tuple<>;
};

template <> struct packetReturn<SPEED> {
  using type =
      std::tuple<float, float, float, float>; // motor Mask, speed, direction
};

template <> struct packetSend<POSITION> {
  using type = std::tuple<>;
};

template <> struct packetReturn<POSITION> {
  using type = std::tuple<unsigned int, unsigned int, unsigned int,
                          unsigned int>; // motor Mask, position, direction
};

template <ReadCommandsNC::ReadCommand ID, typename HandlerFunc>
void dispatch_one(const uint8_t *data, HandlerFunc &&handle) {
  using payload = typename ReadCommandsNC::packetReturn<ID>::type;
  payload p;
  unpack_tuple_from_buffer(p, data);
  handle(std::move(p));
}

} // namespace ReadCommandsNC

namespace CommandsNC {

enum class StaticCommand : uint8_t {
  Ping = 0x01,
};

template <auto u> struct CmdInter {
  constexpr static uint8_t instruction = 0;
  using sending = void;
  using returning = void;
};

template <StaticCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = static_cast<uint8_t>(ID);
  constexpr static uint8_t id = 0;
  constexpr static bool hasID = false;
  using sending = std::tuple<>;
  using returning = std::tuple<>;
};

template <WriteCommandsNC::WriteCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = 2;
  constexpr static uint8_t id = ID;
  constexpr static bool hasID = true;
  using sending = typename WriteCommandsNC::packetSend<ID>::type;
  using returning = typename WriteCommandsNC::packetReturn<ID>::type;
};

template <ReadCommandsNC::ReadCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = 3;
  constexpr static uint8_t id = ID;
  constexpr static bool hasID = true;
  using sending = typename ReadCommandsNC::packetSend<ID>::type;
  using returning = typename ReadCommandsNC::packetReturn<ID>::type;
};

struct Command {

  virtual void pack(uint8_t *buffer) { (void)buffer; }
  virtual size_t getPckSize() { return 0; }
  virtual uint8_t getInst() { return 0; }
  virtual uint8_t getID() { return 0; }
  virtual bool hasID() { return false; }
  virtual ~Command() = default;
};

template <typename T> struct GeneralInstruction : Command {

  using packet = typename T::sending;
  packet info;
  constexpr static size_t size = std::tuple_size<packet>::value;

  constexpr static uint8_t id = T::id;

  GeneralInstruction(packet in = packet{}) : info{in} {}

  void pack(uint8_t *buffer) override {

    if constexpr (T::hasID) {
      buffer[0] = static_cast<uint8_t>(id);
      pack_tuple_to_buffer(info, buffer + 1);
    } else {
      pack_tuple_to_buffer(info, buffer);
    }
  }

  size_t getPckSize() override {
    if constexpr (T::hasID) {
      return size + 1;
    } else {
      return size;
    }
  }

  uint8_t getID() override { return id; }

  bool hasID() override { return T::hasID; }

  uint8_t getInst() override { return T::instruction; }
};

template <StaticCommand T> using MiscInst = GeneralInstruction<CmdInter<T>>;

template <WriteCommandsNC::WriteCommand T>
using WriteInst = GeneralInstruction<CmdInter<T>>;

template <ReadCommandsNC::ReadCommand T>
using ReadInst = GeneralInstruction<CmdInter<T>>;

} // namespace CommandsNC
using namespace CommandsNC;

class Protocol_Handler_I2C {
public:
  enum class Error_State {
    NONE,
    INVALID_CONFIG,
    OPEN_FAILED,
    IOCTL_FAILED,
    IO_ERROR
  };

  using header =
      std::tuple<uint8_t, uint8_t, uint8_t>; // header, command, length

  using tail = std::tuple<uint8_t>; // checksum

  std::unordered_map<ReadCommandsNC::ReadCommand,
                     std::function<void(const uint8_t *)>>
      read_callbacks{};
  std::unordered_map<WriteCommandsNC::WriteCommand,
                     std::function<void(const uint8_t *)>>
      write_callbacks{};

  explicit Protocol_Handler_I2C(rclcpp::Logger log,
                                const std::string &device_in = "",
                                int slave_addr_in = 0x00)
      : device{device_in}, slave_addr{slave_addr_in}, logger{log} {
    if (!device.empty()) {
      // Attempt connection only when a device string was provided.
      connect();
    }

    instruction_callback.emplace(1, [](const uint8_t *pckg) { (void)pckg; });

    instruction_callback.emplace(2, [this](const uint8_t *pckg) {
      uint8_t id = pckg[0];
      auto it =
          write_callbacks.find(static_cast<WriteCommandsNC::WriteCommand>(id));
      if (it != write_callbacks.end()) {
        it->second(pckg + 1);
      }
    });

    // Allow default construction so this type can be used as a member-by-value
    // (the explicit constructor has default parameters so default construction
    // is already supported).
    instruction_callback.emplace(3, [this](const uint8_t *pckg) {
      uint8_t id = pckg[0];
      auto it =
          read_callbacks.find(static_cast<ReadCommandsNC::ReadCommand>(id));
      if (it != read_callbacks.end()) {
        it->second(pckg + 1);
      }
    });
  }

  // Initialize an existing (default-constructed) handler and attempt to
  // connect.
  bool init(const std::string &device_in, int slave_addr_in) {
    device = device_in;
    slave_addr = slave_addr_in;
    return connect();
  }

  // Attempt a reconnect: close any existing fd and try to connect again.
  // Returns true if connection is established.
  bool reconnect() {
    if (device.empty()) {
      error_state = Error_State::INVALID_CONFIG;
      return false;
    }
    return connect();
  }

  ~Protocol_Handler_I2C() noexcept { closeI2C(); }

  bool connect() {
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

  bool addCommand(std::unique_ptr<Command> &&input) {
    if (sending.size() < max_queue) {
      sending.push(std::move(input));
      return true;
    }
    return false;
  }

  // TODO: add sequencing to the protocol
  bool
  sendQueue(std::chrono::milliseconds timeout = std::chrono::milliseconds(5)) {
    if (!connected())
      return false;

    size_t bytes{0};

    using clock = std::chrono::steady_clock;
    auto deadline = clock::now();

    // It can send a little more than a 100 bytes beware, im ok with this
    // behaviour tho (same with the deadline)
    while (!sending.empty() &&
           bytes + sending.front()->getPckSize() +
                   std::tuple_size<header>::value +
                   std::tuple_size<tail>::value <
               100 &&
           clock::now() < deadline + timeout) {

      auto expectedsize = sending.front()->getPckSize();
      auto sent = sendNext();
      if (sent < expectedsize) {
        return false;
      }
      bytes += sent;
    }

    return true;
  }

  // Read pending messages until there are none left. Returns true if at least
  // one message was dispatched, false on I/O error or if no messages were
  // available.
  bool readPending(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(5)) {
    if (!connected())
      return false;

    using clock = std::chrono::steady_clock;

    bool dispatched_any = false;
    for (auto deadline = clock::now(); clock::now() < deadline + timeout;) {
      auto res = readOneMessage();
      if (res == ReadResult::OK_DISPATCHED) {
        RCLCPP_DEBUG(logger, "Message dispatched successfully");
        dispatched_any = true;
        continue; // try reading more
      }
      if (res == ReadResult::NO_MESSAGE) {
        RCLCPP_DEBUG(logger, "No message available to read");
        break; // nothing more to read
      }
      if (res == ReadResult::NO_SYNC) {
        RCLCPP_WARN(logger, "Could not sync to message HEADER");
        break; // nothing more to read
      }
      if (res == ReadResult::CRC_MISMATCH) {
        RCLCPP_WARN(logger, "CRC mismatch for received message, dropping it");
        // Skip and attempt to continue reading next message
        continue;
      }
      // ERROR_IO
      RCLCPP_ERROR(logger, "I/O error occurred while reading message");
      return false;
    }

    return dispatched_any;
  }

  bool connected() const { return i2c_fd > 0; }

  // Error_State getErrorState() const { return error_state; }

  const std::string &getDevice() const { return device; }

  int getSlaveAddress() const { return slave_addr; }

private:
  enum class ReadResult {
    OK_DISPATCHED,
    NO_MESSAGE,
    NO_SYNC,
    CRC_MISMATCH,
    ERROR_IO
  };

  size_t sendNext() {
    if (!connected())
      return 0;
    if (sending.empty())
      return 0;

    // TODO: get rid of std::vector from this function (and maybe also read)
    auto size = sending.front()->getPckSize();
    constexpr auto headerSize = std::tuple_size<header>::value;
    constexpr auto tailSize = std::tuple_size<tail>::value;
    std::vector<uint8_t> buffer(sending.front()->getPckSize() + headerSize +
                                tailSize);

    auto headerSend =
        make_msg_from_args(header{}, 0xAA, sending.front()->getInst(), size);
    pack_tuple_to_buffer(headerSend, buffer.data());
    sending.front()->pack(buffer.data() + headerSize);

    auto tail_tuple = make_msg_from_args(
        tail{}, calcCRC(buffer.data(), headerSize + size, 0));
    pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + size);

    // RCLCPP_DEBUG(logger, "Sending instruction 0x%02X with payload size %zu
    // (total packet size %zu)",
    //             sending.front()->getInst(), size, buffer.size());

    auto sent = writeData(buffer.data(), buffer.size());
    if (sent == buffer.size()) {
      sending.pop();
    }
    return sent;
  }

  ReadResult readOneMessage() {
    if (!connected())
      return ReadResult::ERROR_IO;

    uint8_t header_buf[4];
    header_buf[0] = 0xAA;

    // Wait for sync byte; if none found within timeout, return NO_MESSAGE
    if (!syncMessage()) {
      return ReadResult::NO_SYNC;
    }

    // Read remaining header bytes
    if (readData(header_buf + 1, sizeof(header_buf) - 1) !=
        sizeof(header_buf) - 1) {
      error_state = Error_State::IO_ERROR;
      closeI2C();
      return ReadResult::ERROR_IO;
    }

    using headerScan = tuple_push_back_t<header, uint8_t>;
    headerScan recv_header{};
    unpack_tuple_from_buffer(recv_header, header_buf);

    // Ignore heartbeat/empty packages
    if (recv_header ==
        headerScan{0xAA, 0x00, 0x00, calcCRC({0xAA, 0x00, 0x00}, 0)}) {
      return ReadResult::NO_MESSAGE;
    }

    uint8_t inst = std::get<1>(recv_header);
    uint8_t length = std::get<2>(recv_header);

    std::vector<uint8_t> out_buffer(length + 1);

    if (length > 0) {
      out_buffer[0] = std::get<3>(recv_header);
      if (readData(out_buffer.data() + 1, length) !=
          static_cast<size_t>(length)) {
        error_state = Error_State::IO_ERROR;
        closeI2C();
        return ReadResult::ERROR_IO;
      }
      RCLCPP_DEBUG(logger,
                   "Payload read for instruction 0x%02X: first 4 bytes (or "
                   "full payload if smaller) 0x%02X 0x%02X 0x%02X 0x%02X",
                   inst, out_buffer[0],
                   out_buffer.size() > 1 ? out_buffer[1] : 0,
                   out_buffer.size() > 2 ? out_buffer[2] : 0,
                   out_buffer.size() > 3 ? out_buffer[3] : 0);
    } else {
      RCLCPP_DEBUG(logger, "Payload read for instruction 0x%02X: no payload",
                   inst);
    }

    uint8_t recv_crc =
        out_buffer[length]; // CRC is the byte immediately following the payload

    RCLCPP_DEBUG(logger, "Received CRC: 0x%02X", recv_crc);

    uint8_t header_crc = calcCRC(header_buf, sizeof(header_buf), 0);
    if (recv_crc != calcCRC(out_buffer.data(), length, header_crc)) {
      // CRC mismatch, drop message
      return ReadResult::CRC_MISMATCH;
    }

    RCLCPP_DEBUG(logger, "Received instruction 0x%02X with payload size %d",
                 inst, length);
    // Dispatch the message
    dispatchInput(inst, out_buffer.data());
    return ReadResult::OK_DISPATCHED;
  }

  size_t
  writeData(const uint8_t *data, size_t length,
            std::chrono::milliseconds timeout = std::chrono::milliseconds{1}) {
    if (!connected())
      return 0;

    using clock = std::chrono::steady_clock;
    auto deadline = clock::now() + timeout;

    size_t total{0};

    RCLCPP_DEBUG(logger, "Writing a message with %zu bytes", length);
    while (total < length) {

      size_t available = std::min(INTERNAL_SEND_SIZE, length - total);

      std::array<uint8_t, INTERNAL_SEND_SIZE> buf;
      buf.fill(0xBB);
      memcpy(buf.data(), data, available);

      struct i2c_msg msg {};
      msg.addr = slave_addr;
      msg.flags = 0;
      msg.len = static_cast<__u16>(INTERNAL_SEND_SIZE);
      msg.buf = buf.data();

      struct i2c_rdwr_ioctl_data ioctl_data {};
      ioctl_data.msgs = &msg;
      ioctl_data.nmsgs = 1;

      int ret = ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
      if (ret != static_cast<int>(ioctl_data.nmsgs)) {
        closeI2C();
        error_state = Error_State::IO_ERROR;
        return total;
      }

      total += available;

      if (clock::now() > deadline) {
        RCLCPP_WARN(logger,
                    "Send timeout from device %s at address %d: expected %zu "
                    "bytes, got %zu",
                    device.c_str(), slave_addr, length, total);
        break;
      }
    }

    return total;
  }

  size_t readData(uint8_t *buffer, size_t length,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds{
                      2}) {
    if (!connected() || !buffer || length == 0)
      return 0;

    using clock = std::chrono::steady_clock;
    auto deadline = clock::now() + timeout;

    size_t total = 0;

    while (total < length) {

      // Si el buffer interno está vacío → leer otro chunk de 8
      if (internal_pos >= INTERNAL_BUF_SIZE) {

        RCLCPP_DEBUG(logger, "Reading new chunk...");

        uint8_t skipmsg[4]{0xAA, 0XFF, INTERNAL_BUF_SIZE, 0x00};
        skipmsg[3] = calcCRC(skipmsg, 3, 0);

        struct i2c_msg msg[2]{};
        msg[0].addr = slave_addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = INTERNAL_BUF_SIZE;
        msg[0].buf = internal_buf;

        msg[1].addr = slave_addr;
        msg[1].flags = 0;
        msg[1].len = 4;
        msg[1].buf = skipmsg;

        struct i2c_rdwr_ioctl_data ioctl_data {};
        ioctl_data.msgs = msg;
        ioctl_data.nmsgs = 2;

        int ret = ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
        if (ret != static_cast<int>(ioctl_data.nmsgs)) {
          closeI2C();
          error_state = Error_State::IO_ERROR;
          return total;
        }

        internal_pos = 0;

        // print all the buffer to debug
        RCLCPP_DEBUG(logger,
                     "Read chunk of %zu bytes: 0x%02X 0x%02X 0x%02X 0x%02X "
                     "0x%02X 0x%02X 0x%02X 0x%02X",
                     INTERNAL_BUF_SIZE, internal_buf[0], internal_buf[1],
                     internal_buf[2], internal_buf[3], internal_buf[4],
                     internal_buf[5], internal_buf[6], internal_buf[7]);
      }

      // Copiar desde el buffer interno al buffer del usuario
      size_t available = INTERNAL_BUF_SIZE - internal_pos;
      size_t to_copy = std::min(available, length - total);

      std::memcpy(buffer + total, internal_buf + internal_pos, to_copy);

      internal_pos += to_copy;
      total += to_copy;

      if (clock::now() > deadline) {
        RCLCPP_WARN(logger,
                    "Read timeout from device %s at address %d: expected %zu "
                    "bytes, got %zu",
                    device.c_str(), slave_addr, length, total);
        break;
      }
    }

    return total;
  }

  void closeI2C() {
    if (i2c_fd >= 0) {
      ::close(i2c_fd);
      i2c_fd = -1;

      // Empty the queue
      std::queue<std::unique_ptr<Command>> empty;
      std::swap(sending, empty);

      RCLCPP_INFO(logger, "Closed connection to %s at %d", device.c_str(),
                  slave_addr);

      error_state = Error_State::OPEN_FAILED;
    }
  }

  bool isValidConfig() const noexcept {
    return !device.empty() && slave_addr >= 0x03 && slave_addr <= 0x77;
  }

  // CRC-8-ATM
  uint8_t calcCRC(uint8_t data, uint8_t crc) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
    return crc;
  }

  uint8_t calcCRC(const uint8_t *data, size_t len, uint8_t crc = 0) {
    for (size_t i = 0; i < len; i++)
      crc = calcCRC(data[i], crc);
    return crc;
  }

  uint8_t calcCRC(std::initializer_list<uint8_t> inputs, uint8_t crc) {
    for (auto data : inputs)
      crc = calcCRC(data, crc);
    return crc;
  }

  bool
  syncMessage(std::chrono::milliseconds time = std::chrono::milliseconds(3)) {
    using clock = std::chrono::steady_clock;
    auto deadline = clock::now() + time;
    uint8_t header;
    while (clock::now() < deadline) {
      if (readData(&header, 1, std::chrono::milliseconds(1)) != 1) {
        return false;
      }
      if (header == 0xAA) {
        return true;
      }
    }
    RCLCPP_INFO(logger, "Sync byte not found within timeout");
    return false;
  }

  void dispatchInput(uint8_t inst, uint8_t *pckg) {
    // TODO: register time of incoming messages
    auto it = instruction_callback.find(inst);
    if (it != instruction_callback.end()) {
      it->second(pckg);
    }
  }

  // Read buffer
  // Do not set INTERNAL_SEND_SIZE to 4 or you will be subject to super rare and
  // obscure bugs
  static constexpr size_t INTERNAL_SEND_SIZE = 16;
  static constexpr size_t INTERNAL_BUF_SIZE = 8;

  uint8_t internal_buf[INTERNAL_BUF_SIZE];
  // start "full" to trigger initial read
  size_t internal_pos = INTERNAL_BUF_SIZE;

  constexpr static unsigned int max_queue{50};

  std::string device;
  int i2c_fd{-1}, slave_addr{-1};
  Error_State error_state{Error_State::OPEN_FAILED};

  std::queue<std::unique_ptr<Command>> sending;

  std::unordered_map<uint8_t, std::function<void(const uint8_t *)>>
      instruction_callback{};

  rclcpp::Logger logger;
};

class HardwareDriverNode : public rclcpp::Node {
public:
  HardwareDriverNode() : Node("hardware_node") {
    // Parameters
    this->declare_parameter<std::string>("i2c_port", "/dev/i2c-7");
    this->declare_parameter<int>("i2c_address", 0x30);
    this->declare_parameter<int>("flipper_revolution", 400);
    this->declare_parameter<std::vector<std::string>>(
        "joint_names", {"flipper_0", "flipper_1", "flipper_2", "flipper_3"});
    this->declare_parameter("steppers", 4);

    std::string i2c_port_;
    int slave_addr_{};

    this->get_parameter("i2c_port", i2c_port_);
    this->get_parameter("i2c_address", slave_addr_);
    this->get_parameter("flipper_revolution", steps_per_revolution);
    this->get_parameter("joint_names", joint_names);
    this->get_parameter("steppers", steppers);

    stepper_micro.init(i2c_port_, slave_addr_);

    // init joint state storage from configured joint names
    const size_t n = joint_names.size();
    feedback_joint_positions.assign(n, 0.0);
    feedback_joint_velocities.assign(n, 0.0);
    feedback_joint_efforts.assign(n, 0.0);

    in_joint_positions.assign(n, 0.0);
    in_joint_velocities.assign(n, 0.0);
    in_joint_efforts.assign(n, 0.0);

    // Initialize last-sent trackers so first change is detected
    last_sent_pos_int_.assign(n, std::numeric_limits<int32_t>::min());
    last_sent_spd_int_.assign(n, std::numeric_limits<int32_t>::min());

    generateCallbacks();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "hardware_node/cmd_vel", qos_cmd_,
        std::bind(&HardwareDriverNode::cmdVelCallback, this,
                  std::placeholders::_1));

    joint_cmd_sub_ = this->create_subscription<hardware::msg::JointControl>(
        "hardware_node/joint_command", qos_cmd_,
        std::bind(&HardwareDriverNode::jointCommandCallback, this,
                  std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "hardware_node/joint_states", qos_feedback_);
  }

  void tick() {

    using positionW = WriteInst<WriteCommandsNC::POSITION>;
    using speedW = WriteInst<WriteCommandsNC::SPEED>;

    stepper_micro.sendQueue();
    stepper_micro.readPending();
    RCLCPP_DEBUG(this->get_logger(), "Passed the sendings and readings");

    switch (tick_state_) {
    case TickState::IDLE:

      if (!stepper_micro.connected()) {
        tick_state_ = TickState::ERROR_RECOVERY;
      }

      if (ticks_since_poll_++ >= poll_interval_ticks_) {
        ticks_since_poll_ = 0;
        tick_state_ = TickState::PREPARE_COMMANDS;
      } else {
        // detect any change requiring immediate command
        bool changed = false;
        for (size_t i = 0;
             i < static_cast<size_t>(steppers) && i < in_joint_positions.size();
             ++i) {
          int32_t pos_key = static_cast<int32_t>(std::llround(
              in_joint_positions[i] / M_2_PI * steps_per_revolution));
          int32_t spd_key = static_cast<int32_t>(std::llround(
              in_joint_velocities[i] / M_2_PI * steps_per_revolution));
          if (pos_key != last_sent_pos_int_[i] ||
              spd_key != last_sent_spd_int_[i]) {
            changed = true;
            break;
          }
        }
        if (changed) {
          tick_state_ = TickState::PREPARE_COMMANDS;
        }
      }
      break;

    case TickState::PREPARE_COMMANDS: {
      // Group identical converted inputs into masks
      std::unordered_map<int32_t, uint8_t> pos_groups; // pos_key -> mask
      std::unordered_map<int32_t, uint8_t> spd_groups; // spd_key -> mask
      std::unordered_map<int32_t, float>
          spd_value; // spd_key -> representative float

      for (size_t i = 0;
           i < static_cast<size_t>(steppers) && i < in_joint_positions.size();
           ++i) {
        int32_t pos_key = static_cast<int32_t>(std::llround(
            in_joint_positions[i] / M_2_PI * steps_per_revolution));
        int32_t spd_key = static_cast<int32_t>(std::llround(
            in_joint_velocities[i] / M_2_PI * steps_per_revolution));

        uint8_t bit = static_cast<uint8_t>(1u << i);
        pos_groups[pos_key] |= bit;

        spd_groups[spd_key] |= bit;
        // store representative float value (convert back)
        float spd_f = static_cast<float>(in_joint_velocities[i] / M_2_PI *
                                         steps_per_revolution);
        spd_value.emplace(spd_key, spd_f);
      }

      // Enqueue position commands (one per unique converted value)
      for (auto &p : pos_groups) {
        int32_t value = p.first;
        uint8_t mask = p.second;
        auto pkt = positionW::packet{mask, value};
        stepper_micro.addCommand(std::make_unique<positionW>(pkt));
      }

      // Enqueue speed commands
      for (auto &s : spd_groups) {
        float value = spd_value[s.first];
        uint8_t mask = s.second;
        auto pkt = speedW::packet{mask, value};
        stepper_micro.addCommand(std::make_unique<speedW>(pkt));
      }

      // update last_sent keys
      for (size_t i = 0;
           i < static_cast<size_t>(steppers) && i < in_joint_positions.size();
           ++i) {
        last_sent_pos_int_[i] = static_cast<int32_t>(std::llround(
            in_joint_positions[i] / M_2_PI * steps_per_revolution));
        last_sent_spd_int_[i] = static_cast<int32_t>(std::llround(
            in_joint_velocities[i] / M_2_PI * steps_per_revolution));
      }

      stepper_micro.addCommand(
          std::make_unique<ReadInst<ReadCommandsNC::POSITION>>());
      stepper_micro.addCommand(
          std::make_unique<ReadInst<ReadCommandsNC::SPEED>>());

      wait_counter_ = 0;
      tick_state_ = TickState::WAIT_RESPONSE;
    } break;

    case TickState::WAIT_RESPONSE:
      // give device some time (a few ticks) to respond
      if (++wait_counter_ >= max_wait_ticks_) {
        tick_state_ = TickState::PUBLISH_STATE;
      }
      break;

    case TickState::PUBLISH_STATE:
      publishJointFeedback();
      tick_state_ = TickState::IDLE;
      break;

    case TickState::ERROR_RECOVERY:
      if (stepper_micro.connected()) {
        tick_state_ = TickState::IDLE;
      }

      if (wait_counter_ >= max_wait_ticks_) {
        if (stepper_micro.reconnect()) {
          tick_state_ = TickState::IDLE;
        }
        wait_counter_ = 0;
      }

      break;
    }
  }

private:
  void generateCallbacks() {
    stepper_micro.read_callbacks.emplace(
        ReadCommandsNC::ReadCommand::POSITION, [this](const uint8_t *data) {
          ReadCommandsNC::dispatch_one<ReadCommandsNC::ReadCommand::POSITION>(
              data, [&](const auto &info) {
                std::apply(
                    [&](const auto &...args) {
                      size_t i{0};
                      ((feedback_joint_positions[i++] =
                            static_cast<double>(args) * steps_per_revolution /
                            M_2_PI),
                       ...);
                      updated_pos = true;
                    },
                    info);
              });
        });

    stepper_micro.read_callbacks.emplace(
        ReadCommandsNC::ReadCommand::SPEED, [this](const uint8_t *data) {
          ReadCommandsNC::dispatch_one<ReadCommandsNC::SPEED>(
              data, [&](const auto &info) {
                std::apply(
                    [&](const auto &...args) {
                      size_t i{0};
                      ((feedback_joint_velocities[i++] = static_cast<double>(
                            args * steps_per_revolution / M_2_PI)),
                       ...);
                      updated_vel = true;
                    },
                    info);
              });
        });
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    (void)msg;
  }

  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg) {
    std::lock_guard<std::mutex> lock{joints};
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {

      const std::string &name = msg->joint_names[i];
      auto it = std::find(joint_names.begin(), joint_names.end(), name);

      if (it == joint_names.end()) {
        RCLCPP_WARN(this->get_logger(), "Unknown joint name: %s", name.c_str());
        continue;
      }

      auto joint_index = std::distance(joint_names.begin(), it);
      if (joint_index < steppers) {

        in_joint_positions[joint_index] = msg->position[i];
        in_joint_velocities[joint_index] = msg->velocity[i];

        continue;
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid joint name format: %s",
                    msg->joint_names[i].c_str());
      }
    }
  }

  void publishJointFeedback() {
    // TODO: add timeout
    if (updated_pos || updated_vel) {
      std::lock_guard<std::mutex> lock{joints};
      // Publish JointState with current stored values
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->now();
      msg.name = joint_names;
      msg.position = feedback_joint_positions;
      msg.velocity = feedback_joint_velocities;
      msg.effort = feedback_joint_efforts;
      joint_state_pub_->publish(msg);

      updated_pos = updated_vel = false;
    }
  }

  // I2C protocol handler (owned)
  Protocol_Handler_I2C stepper_micro{this->get_logger()};

  // Steppers
  int steps_per_revolution{400};
  int steppers{4};

  std::vector<std::string> joint_names{"flipper_0", "flipper_1", "flipper_2",
                                       "flipper_3"};
  bool updated_pos{false};
  std::vector<double> feedback_joint_positions;
  bool updated_vel{false};
  std::vector<double> feedback_joint_velocities;
  std::vector<double> feedback_joint_efforts;

  std::vector<double> in_joint_positions;
  std::vector<double> in_joint_velocities;
  std::vector<double> in_joint_efforts;

  // Tick state machine
  enum class TickState {
    IDLE,
    PREPARE_COMMANDS,
    WAIT_RESPONSE,
    PUBLISH_STATE,
    ERROR_RECOVERY
  };

  TickState tick_state_{TickState::IDLE};
  int wait_counter_{0};
  const int max_wait_ticks_{3};

  int ticks_since_poll_{0};
  const int poll_interval_ticks_{3};

  std::vector<int32_t> last_sent_pos_int_;
  std::vector<int32_t> last_sent_spd_int_;

  // Ros configuration
  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::mutex joints;

  // rclcpp::TimerBase::SharedPtr poll_timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HardwareDriverNode>();

  // Rate del loop principal (controla TODO el bus)
  constexpr int LOOP_HZ = 50;
  rclcpp::Rate rate(LOOP_HZ);

  RCLCPP_INFO(node->get_logger(), "DC Motors node started");

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    node->tick();

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}