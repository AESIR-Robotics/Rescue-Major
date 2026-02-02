#include <cstdint>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <fcntl.h>      // open
#include <sys/types.h>
#include <tuple>
#include <unistd.h>     // write, close
#include <termios.h>    // termios


#include <type_traits>

#include "geometry_msgs/msg/twist.hpp"
// #include "control_msgs/msg/joint_jog.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware/msg/joint_control.hpp"

/* 
// Designed this for runtype buffer builders
// Then remembered that messages are known at compile time making this unnecessary
// I do not have the heart to delete it though
// Might be useful for run time message testing

class BufferBuilder{
public:

  using info_unit = std::variant<int8_t, uint8_t,int16_t,
                              uint16_t, int32_t, uint32_t,
                              int64_t, uint64_t, float, double, char>;

  struct TypeInfo {
    char fmt;
    size_t size;
    info_unit (*make_default)();
  };

  static info_unit make_i8()  { return int8_t{}; }
  static info_unit make_u8()  { return uint8_t{}; }
  static info_unit make_i16() { return int16_t{}; }
  static info_unit make_u16() { return uint16_t{}; }
  static info_unit make_i32() { return int32_t{}; }
  static info_unit make_u32() { return uint32_t{}; }
  static info_unit make_i64() { return int64_t{}; }
  static info_unit make_u64() { return uint64_t{}; }
  static info_unit make_f32() { return float{}; }
  static info_unit make_f64() { return double{}; }
  static info_unit make_char(){ return char{}; }

  const static std::unordered_map<char, TypeInfo> type_table;

  BufferBuilder(const std::string & format): fmt{format} {
    buffer.reserve(16);
    setFormat(fmt);
  }

  bool setFormat(const std::string & new_format){
    if (!std::regex_match(new_format, regform)) {
      fmt = "";
      return false;
    }
    fmt = new_format;
    elements.clear();
    elements.reserve(fmt.size());
    for(auto x : fmt){
      elements.push_back(type_table.at(x).make_default());
    }
    buffer.clear();
    return true;
  }

  bool pack(const std::vector<info_unit> &information){
    if(elements.size() != information.size()){
      return false;
    }

    for (size_t i = 0; i < information.size(); ++i) {
      if (elements[i].index() != information[i].index()) {
        return false;
      }
    }

    elements = information;
    compileBuffer();
    return true;
  }

  const uint8_t * getBuffer() const {
    if(fmt == ""){
      return nullptr;
    }
    return buffer.data();
  }

  size_t getBufSize() const {
    return buffer.size();
  }

  std::string getFormat(){
    return fmt;
  }

private:
  std::string fmt;
  std::vector<uint8_t> buffer;
  std::vector<info_unit> elements;

  inline static std::regex regform{R"([bBhHiIqQfdc]*)"};

  bool compileBuffer(){
    buffer.clear();
    for(const auto & elem : elements){
      std::visit([this](auto&& arg){
        using T = std::decay_t<decltype(arg)>;
        const uint8_t * ptr = reinterpret_cast<const uint8_t*>(&arg);
        buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
      }, elem);
    }
    return true;
  }

};
  
const std::unordered_map<char, BufferBuilder::TypeInfo>  BufferBuilder::type_table{
      { 'b', {'b', 1, BufferBuilder::make_i8  } },
      { 'B', {'B', 1, BufferBuilder::make_u8  } },
      { 'h', {'h', 2, BufferBuilder::make_i16 } },
      { 'H', {'H', 2, BufferBuilder::make_u16 } },
      { 'i', {'i', 4, BufferBuilder::make_i32 } },
      { 'I', {'I', 4, BufferBuilder::make_u32 } },
      { 'q', {'q', 8, BufferBuilder::make_i64 } },
      { 'Q', {'Q', 8, BufferBuilder::make_u64 } },
      { 'f', {'f', 4, BufferBuilder::make_f32 } },
      { 'd', {'d', 8, BufferBuilder::make_f64 } },
      { 'c', {'c', 1, BufferBuilder::make_char} },
  };
*/

// Compute total byte size of a tuple (sum of sizeof each element type)
template<typename... Args>
constexpr size_t tuple_size(const std::tuple<Args...>&) {
  return (sizeof(Args) + ... + 0);
}

// Helpers to pack/unpack a tuple into a contiguous byte buffer
// Recursive tuple pack/unpack helpers (safer and easier to reason about)
template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
pack_tuple_recursive(const std::tuple<Args...>&, uint8_t* /*buffer*/, size_t& /*offset*/) {
  // end recursion
}

template <size_t I = 0, typename... Args>
std::enable_if_t<I < sizeof...(Args), void>
pack_tuple_recursive(const std::tuple<Args...>& tup, uint8_t* buffer, size_t& offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  std::memcpy(buffer + offset, &std::get<I>(tup), sizeof(T));
  offset += sizeof(T);
  pack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void pack_tuple_to_buffer(const std::tuple<Args...>& tuple, uint8_t* buffer) {
  size_t offset = 0;
  pack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
unpack_tuple_recursive(std::tuple<Args...>&, const uint8_t* /*buffer*/, size_t& /*offset*/) {
  // end recursion
}

template <size_t I = 0, typename... Args>
std::enable_if_t<I < sizeof...(Args), void>
unpack_tuple_recursive(std::tuple<Args...>& tup, const uint8_t* buffer, size_t& offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  std::memcpy(&std::get<I>(tup), buffer + offset, sizeof(T));
  offset += sizeof(T);
  unpack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template<typename... Args>
void unpack_tuple_from_buffer(std::tuple<Args...>& tuple, const uint8_t* buffer) {
  size_t offset = 0;
  unpack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}


template<typename Tuple, typename... Args>
struct args_match_tuple;

template<typename... Ts, typename... Args>
struct args_match_tuple<std::tuple<Ts...>, Args...>
{
  static constexpr bool value =
    (sizeof...(Ts) == sizeof...(Args)) &&
    (std::is_constructible_v<Ts, Args> && ...);
};

template<typename... TupleArgs, typename... Args>
auto make_msg_from_args(std::tuple<TupleArgs...>, const Args &... args) {
  static_assert(args_match_tuple<std::tuple<TupleArgs...>, Args...>::value,
                "Argument types do not match tuple types");
  
  // Static cast of every compatible argument to create std::tuple<TupleArgs...>
  return std::tuple<TupleArgs...>{(static_cast<TupleArgs>(args))...};
}

namespace WriteCommandsNC {

  enum WriteCommand : uint8_t {
      DIRECTION = 0x00,
      SPEED = 0x01,
      POSITION = 0x02
    };

  template<WriteCommand CMD>
  struct packet {
      using type = void;
  };

  template<>
  struct packet<DIRECTION> {
      using type = std::tuple<uint8_t, bool>; // motor Mask, direction
  };

  template<>
  struct packet<SPEED> {
      using type = std::tuple<uint8_t, float, bool>; // motor Mask, speed, direction
  };

  template<>
  struct packet<POSITION> {
      using type = std::tuple<uint8_t, float, bool>; // motor Mask, position, direction
  };

} // namespace WriteCommands

namespace ReadCommandsNC {

  enum ReadCommand : uint8_t {
    DIRECTION = 0x00,
    SPEED = 0x01,
    POSITION = 0x02
  };

  template<ReadCommand CMD>
  struct packet {
    using type = void;
  };

  template<>
  struct packet<DIRECTION> {
    using type = std::tuple<bool, bool, bool, bool>; // motor Mask, direction
  };

  template<>
  struct packet<SPEED> {
    using type = std::tuple<float, float, float, float>; // motor Mask, speed, direction
  };

  template<>
  struct packet<POSITION> {
    using type = std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>; // motor Mask, position, direction
  };

} // namespace WriteCommands

class Protocol_Handler_I2C{
  public:

  enum class Error_State {
    NONE,
    INVALID_CONFIG,
    OPEN_FAILED,
    IOCTL_FAILED,
    IO_ERROR
  };

  using header = std::tuple<uint8_t, uint8_t, uint8_t>; // header, command, length
  using tail = std::tuple<uint8_t>; // checksum

  explicit Protocol_Handler_I2C(const std::string & device_in, int slave_addr_in):
  device{device_in}, slave_addr{slave_addr_in}
  {
    connect();
  }

  ~Protocol_Handler_I2C() noexcept{
    closeI2C();
  }

  bool connect(){
    std::lock_guard<std::mutex> lock(lock_i2c);
    closeI2C_nolock();           
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
      closeI2C_nolock();
      return false;
    }
    return true;
  }

  template<WriteCommandsNC::WriteCommand ID, typename... Args>
  bool writeInfo(Args... args){
    
    using Packet = typename WriteCommandsNC::packet<ID>::type;

    static_assert(!std::is_void<Packet>::value, "Invalid Write Command ID");

    auto sendPacket = make_msg_from_args(Packet{}, args...);
    constexpr auto packetSize = tuple_size(decltype(sendPacket){});

    auto sendHeader = make_msg_from_args(header{}, 0xAA, 0x01, packetSize);
    constexpr auto headerSize = tuple_size(decltype(sendHeader){});
    
    constexpr auto tailSize = tuple_size(tail{});
    constexpr auto msgsize = headerSize + packetSize + tailSize;

    std::array<uint8_t, msgsize> buffer{};

    pack_tuple_to_buffer(sendHeader, buffer.data());
    pack_tuple_to_buffer(sendPacket, buffer.data() + headerSize);

    // Build tail (CRC) tuple and pack it (use returned tuple, do not ignore)
    auto tail_tuple = make_msg_from_args(tail{}, calcCRC(buffer.data(), msgsize - tailSize, 0));
    pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + packetSize);

    std::lock_guard<std::mutex> lock(lock_i2c);
    return writeData(buffer.data(), buffer.size());
  }

  template<ReadCommandsNC::ReadCommand ID>
  bool sendReadRequest(){
    
    using Packet = typename ReadCommandsNC::packet<ID>::type;

    static_assert(!std::is_void<Packet>::value, "Invalid Read Command ID");

    auto sendHeader = make_msg_from_args(header{}, 0xAA, 0x02, 0x00);
    constexpr auto headerSize = tuple_size(decltype(sendHeader){});
    
    constexpr auto tailSize = tuple_size(tail{});
    constexpr auto msgsize = headerSize + tailSize;

    std::array<uint8_t, msgsize> buffer{};

    pack_tuple_to_buffer(sendHeader, buffer.data());

    auto tail_tuple = make_msg_from_args(tail{}, calcCRC(buffer.data(), msgsize - tailSize, 0));
    pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize);

    std::lock_guard<std::mutex> lock(lock_i2c);
    return writeData(buffer.data(), buffer.size());

  }

  bool readInfo(uint8_t& id, std::vector<uint8_t> & out_buffer){
    if(!connected()) return false;

    out_buffer.clear();
    id = 0;

    std::lock_guard<std::mutex> lock(lock_i2c);
    uint8_t header_buf[3];
    if(!readData(header_buf, sizeof(header_buf))){
      return false;
    }

    header recv_header{};
    unpack_tuple_from_buffer(recv_header, header_buf);

    if(recv_header == header{0xBB, 0XBB, 0XBB}){
      return false;
    }

    if(std::get<0>(recv_header) != 0xAA){
      // Change this to a flush of the I2C buffer later
      // We're holding the lock in this public function, use nolock close
      closeI2C_nolock();
      error_state = Error_State::IO_ERROR;
      return false;
    }

    id = std::get<1>(recv_header);
    uint8_t length = std::get<2>(recv_header);

    out_buffer.resize(length);

    if(!readData(out_buffer.data(), length)){
      return false;
    }

    uint8_t recv_crc = 0;
    if(!readData(&recv_crc, sizeof(recv_crc))){
      return false;
    }

    uint8_t header_crc = calcCRC(header_buf, sizeof(header_buf), 0);

    return recv_crc == calcCRC(out_buffer.data(), length, header_crc);
  }

  bool connected() const {
    std::lock_guard<std::mutex> lock(lock_i2c);
    return error_state == Error_State::NONE;
  }

  Error_State getErrorState() const {
    std::lock_guard<std::mutex> lock(lock_i2c);
    return error_state;
  }

  const std::string & getDevice() const {
    return device;
  }

  int getSlaveAddress() const {
    return slave_addr;
  }

  private:

  bool writeData(const uint8_t * data, size_t length) {
    // Caller must hold lock_i2c before calling this helper.
    if(!connected()) return false;
    ssize_t bytes_written = ::write(i2c_fd, data, length);
    if (bytes_written != static_cast<ssize_t>(length)) {
      // closeI2C_nolock assumes lock is held by caller
      closeI2C_nolock();
      error_state = Error_State::IO_ERROR;
      return false;
    }
    return true;
  }

  bool readData(uint8_t * buffer, size_t length) {
    // Caller must hold lock_i2c before calling this helper.
    if(!connected()) return false;
    ssize_t bytes_read = ::read(i2c_fd, buffer, length);
    if (bytes_read != static_cast<ssize_t>(length)) {
      // closeI2C_nolock assumes lock is held by caller
      closeI2C_nolock();
      error_state = Error_State::IO_ERROR;
      return false;
    }
    return true;
  }

  // Close without taking the mutex; caller must hold lock_i2c
  void closeI2C_nolock() {
    if (i2c_fd >= 0) {
      ::close(i2c_fd);
      i2c_fd = -1;
    }
  }

  // Public close that acquires the mutex
  void closeI2C() {
    std::lock_guard<std::mutex> lock(lock_i2c);
    closeI2C_nolock();
  }

  bool isValidConfig() const noexcept {
    return !device.empty() &&
        slave_addr >= 0x03 &&
        slave_addr <= 0x77;
  }

  uint8_t calcCRC(const uint8_t *data, size_t len, uint8_t c = 0) const {
    for (size_t i = 0; i < len; i++)
      c ^= data[i];
    return c;
  }

  uint8_t calcCRC(const uint8_t data, uint8_t c = 0) const {
      return c ^= data;
  }

  std::string device;
  int i2c_fd{-1}, slave_addr{-1};
  Error_State error_state{Error_State::OPEN_FAILED};
  mutable std::mutex lock_i2c;

};


class HardwareDriverNode : public rclcpp::Node
{
public:
  HardwareDriverNode()
  : Node("hardware_node")
  {

    this->declare_parameter<std::string>("i2c_port", "/dev/i2c-1");
    this->declare_parameter<int>("i2c_address", 0x10);

    this->get_parameter("i2c_port", i2c_port_);
    this->get_parameter("i2c_address", slave_addr_);

    openI2C();

    cmd_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    feedback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cmd_opts;
    cmd_opts.callback_group = cmd_group_;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "hardware_node/cmd_vel", qos_cmd_,
      std::bind(&HardwareDriverNode::cmdVelCallback, this, std::placeholders::_1),
      cmd_opts);

    joint_cmd_sub_ = this->create_subscription<hardware::msg::JointControl>(
      "hardware_node/joint_command", qos_cmd_,
      std::bind(&HardwareDriverNode::jointCommandCallback, this, std::placeholders::_1),
      cmd_opts);


    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hardware_node/joint_states", qos_feedback_);


    feedback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&HardwareDriverNode::pollHardwareFeedback, this),
      feedback_group_);
  }

private:

  void openI2C() {
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
      throw std::runtime_error("No se pudo abrir I2C");
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, 0x10) < 0) {
      throw std::runtime_error("No se pudo seleccionar esclavo");
    }

    RCLCPP_INFO(get_logger(), "I2C listo");
  }

  void closeI2C() {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
      i2c_fd_ = -1;
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){(void)msg;}
  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg){(void)msg;}

  void pollHardwareFeedback(){}

  std::string i2c_port_;
  int i2c_fd_, slave_addr_;
  std::mutex lock_i2c;

  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::CallbackGroup::SharedPtr cmd_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_group_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HardwareDriverNode>();

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4); 
  
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}