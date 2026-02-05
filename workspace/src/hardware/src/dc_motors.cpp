#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <math.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <fcntl.h>      // open
#include <sys/types.h>
#include <tuple>
#include <unistd.h>     // write, close
#include <termios.h>    // termios


#include <type_traits>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
// #include "control_msgs/msg/joint_jog.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware/msg/joint_control.hpp"


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
      using type = std::tuple<uint8_t, float>; // motor Mask, speed
  };

  template<>
  struct packet<POSITION> {
      using type = std::tuple<uint8_t, int32_t>; // motor Mask, position
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

  template<ReadCommandsNC::ReadCommand ID, typename HandlerFunc>
  void dispatch_one(const std::vector<uint8_t>& data, HandlerFunc&& handle) {
    using payload = typename ReadCommandsNC::packet<ID>::type;
    payload p;
    unpack_tuple_from_buffer(p, data.data());
    handle(std::move(p));
  }

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

  explicit Protocol_Handler_I2C(const std::string & device_in = "", int slave_addr_in = 0x00)
    : device{device_in}, slave_addr{slave_addr_in}
  {
    if (!device.empty()){
      // Attempt connection only when a device string was provided.
      connect();
    }
  }

  // Initialize an existing (default-constructed) handler and attempt to connect.
  bool init(const std::string & device_in, int slave_addr_in){
    device = device_in;
    slave_addr = slave_addr_in;
    return connect();
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

    auto sendHeader = make_msg_from_args(header{}, 0xAA, 0x02, 0x01);
    constexpr auto headerSize = tuple_size(decltype(sendHeader){});
    
    constexpr auto tailSize = tuple_size(tail{});
    constexpr auto msgsize = headerSize + 1 + tailSize;

    std::array<uint8_t, msgsize> buffer{};

    pack_tuple_to_buffer(sendHeader, buffer.data());
    pack_tuple_to_buffer(std::make_tuple(static_cast<uint8_t>(ID)), buffer.data() + headerSize);
    auto tail_tuple = make_msg_from_args(tail{}, calcCRC(buffer.data(), msgsize - tailSize, 0));
    pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + 1);

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
    // Parameters
    this->declare_parameter<std::string>("i2c_port", "/dev/i2c-1");
    this->declare_parameter<int>("i2c_address", 0x10);
    this->declare_parameter<int>("flipper_revolution", 400);
    this->declare_parameter<std::vector<std::string>>("joint_names",
      {"flipper_0", "flipper_1", "flipper_2", "flipper_3"});
    this->declare_parameter("steppers", 4);

    std::string i2c_port_;
    int slave_addr_{};

    this->get_parameter("i2c_port", i2c_port_);
    this->get_parameter("i2c_address", slave_addr_);
    this->get_parameter("flipper_revolution", steps_per_revolution);
    this->get_parameter("joint_names", joint_names);
    this->get_parameter("steppers", steppers);

    protocol_handler_.init(i2c_port_, slave_addr_);
    if(protocol_handler_.getErrorState() != Protocol_Handler_I2C::Error_State::NONE){
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to I2C device %s at address 0x%02X",
                   i2c_port_.c_str(), slave_addr_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Protocol handler created for I2C device %s at address 0x%02X",
                  i2c_port_.c_str(), slave_addr_);
    }

    // init joint state storage from configured joint names
    const size_t n = joint_names.size();
    joint_positions_.assign(n, 0.0);
    joint_velocities_.assign(n, 0.0);
    joint_efforts_.assign(n, 0.0);

    cmd_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    feedback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

    generateCallbacks();

    // timer to poll hardware and publish JointState
    poll_timer = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&HardwareDriverNode::pollHardwareFeedback, this),
      feedback_group_);

    read_timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&HardwareDriverNode::readHardwareFeedback, this),
      feedback_group_);
  }

private:

  void generateCallbacks(){
    read_callbacks_.emplace(ReadCommandsNC::ReadCommand::POSITION, [this](const std::vector<uint8_t>& data){
      ReadCommandsNC::dispatch_one<ReadCommandsNC::ReadCommand::POSITION>(data, [&](const auto& info){
        std::apply([&](const auto&... args){
          size_t i{0};
          ((joint_positions_[i++] = static_cast<double>(args) * steps_per_revolution / M_2_PI), ...);
          updated_pos = true;
        }, info);
      });
    });

    read_callbacks_.emplace(ReadCommandsNC::ReadCommand::SPEED, [this](const std::vector<uint8_t>& data){
     ReadCommandsNC::dispatch_one<ReadCommandsNC::SPEED>(data, [&](const auto& info){
      std::apply([&](const auto&... args){
        size_t i{0};
        ((joint_velocities_[i++] = static_cast<double>(args * steps_per_revolution / M_2_PI)),...);
        updated_vel = true;
      }, info);
     });
    });
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){(void)msg;}

  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg){
    if(!protocol_handler_.connected()){
      return;
    }

    for(size_t i = 0; i < msg->joint_names.size(); ++i){

      const std::string& name = msg->joint_names[i];
      auto it = std::find(joint_names.begin(), joint_names.end(), name);

      if(it == joint_names.end()){
        RCLCPP_WARN(this->get_logger(), "Unknown joint name: %s", name.c_str());
        continue;
      }

      auto joint_index = std::distance(joint_names.begin(), it);
      if (joint_index < steppers) {
        int32_t position = static_cast<int32_t>(msg->position[i] / M_2_PI * steps_per_revolution);
        float speed = static_cast<float>(msg->velocity[i] / M_2_PI * steps_per_revolution);

        // Send position command as example
        bool test;
        test = protocol_handler_.writeInfo<WriteCommandsNC::POSITION>(
          1 << joint_index, 
          position
        );

        if(!test){
          RCLCPP_ERROR(this->get_logger(), "Unable to send message position to %s", name.c_str());
          continue;
        }

        test = protocol_handler_.writeInfo<WriteCommandsNC::SPEED>(
          1 << joint_index,
          speed
        );

        if(!test){
          RCLCPP_ERROR(this->get_logger(), "Unable to send message speed to %s", name.c_str());
          continue;
        }

        continue;
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid joint name format: %s", msg->joint_names[i].c_str());
      }

    }

  }

  void pollHardwareFeedback(){
    if(!protocol_handler_.connected()){
      return;
    }
    // Request position and speed frames from device; responses will be
    // processed by readHardwareFeedback when available.
    if(!protocol_handler_.sendReadRequest<ReadCommandsNC::POSITION>()){
      RCLCPP_ERROR(this->get_logger(), "Unable to ask for Position");
      return;
    }
    if(!protocol_handler_.sendReadRequest<ReadCommandsNC::SPEED>()){
      RCLCPP_ERROR(this->get_logger(), "Unable to ask for Speed");
    }
  }

  void readHardwareFeedback(){
    if(!protocol_handler_.connected()){
      return;
    }

    uint8_t id = 0;
    std::vector<uint8_t> buf;

    while(protocol_handler_.readInfo(id, buf)){
      auto incoming = static_cast<ReadCommandsNC::ReadCommand>(id);
      auto it = read_callbacks_.find(incoming);
      if(it != read_callbacks_.end()){
        it->second(buf);
      }
    }

    // Necesary due to the fact that it can fail whilst reading
    if(!protocol_handler_.connected()){
      RCLCPP_ERROR(this->get_logger(), "Unable to read from the microcontroler");
      return;
    }

    if(updated_pos && updated_vel){
      // Publish JointState with current stored values
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->now();
      msg.name = joint_names;
      msg.position = joint_positions_;
      msg.velocity = joint_velocities_;
      msg.effort = joint_efforts_;
      joint_state_pub_->publish(msg);
    }
  }

  // I2C protocol handler (owned)
  Protocol_Handler_I2C protocol_handler_;
  std::unordered_map<ReadCommandsNC::ReadCommand, std::function<void(const std::vector<uint8_t>&)>> read_callbacks_;

  // Steppers
  int steps_per_revolution{400};
  int steppers{4};

  std::vector<std::string> joint_names{"flipper_0", "flipper_1", "flipper_2", "flipper_3"};
  bool updated_pos{false};
  std::vector<double> joint_positions_;
  bool updated_vel{false};
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;

  // Ros configuration
  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::CallbackGroup::SharedPtr cmd_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_group_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr poll_timer;
  rclcpp::TimerBase::SharedPtr read_timer;
};

// TODO: 
// - Change CRC to CRC-8-ATM
// - Handle Read check to the new message
// - Beware of the instruction changes (1 por ping, 2 for write and 3 for read)

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