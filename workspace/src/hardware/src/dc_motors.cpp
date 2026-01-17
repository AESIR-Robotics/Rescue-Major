#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <fcntl.h>      // open
#include <unistd.h>     // write, close
#include <termios.h>    // termios
#include <errno.h>
#include <cstring>
#include <string>
#include <mutex>

class MotorNode : public rclcpp::Node
{
public:
  MotorNode()
  : Node("motor_node"),
    serial_port_("/dev/ttyUSB0"),
    fd_(-1)
  {
    // intentar abrir el puerto serie
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir puerto serial %s: %s",
                   serial_port_.c_str(), std::strerror(errno));
    } else {
      if (!configure_port(fd_, B115200)) {
        RCLCPP_ERROR(this->get_logger(), "Error configurando el puerto serie");
        close(fd_);
        fd_ = -1;
      } else {
        RCLCPP_INFO(this->get_logger(), "Puerto serial %s abierto a 115200 bps",
                    serial_port_.c_str());
      }
    }

    // Suscripción al tópico
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/dc_motors", 10,
      std::bind(&MotorNode::cmd_vel_callback, this, std::placeholders::_1));
  }

  ~MotorNode()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  bool configure_port(int fd, speed_t baud)
  {
    struct termios tty;
    std::memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 bits
    tty.c_cflag |= CLOCAL | CREAD;                 // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);             // no parity
    tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // no flow control

    // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    // blocking read with timeout:
    tty.c_cc[VMIN]  = 0;    // no bytes required
    tty.c_cc[VTIME] = 10;   // timeout in deciseconds (10 -> 1.0s)

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  void motors(int left, int right)
  {
    // misma lógica que en Python:
    if (left < 0)  left = (-left) + 100;
    if (right < 0) right = (-right) + 100;

    // comandos: (11, left) y (12, right)
    send_data(11, static_cast<uint16_t>(left));
    send_data(12, static_cast<uint16_t>(right));
  }

  void send_data(uint8_t command, uint16_t number)
  {
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Puerto serie no disponible, no se envía datos");
      return;
    }

    uint8_t buf[3];
    buf[0] = command;
    buf[1] = static_cast<uint8_t>((number >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(number & 0xFF);

    std::lock_guard<std::mutex> lock(serial_mutex_);
    ssize_t written = write(fd_, buf, sizeof(buf));
    if (written != static_cast<ssize_t>(sizeof(buf))) {
      RCLCPP_ERROR(this->get_logger(), "Error al escribir en serial: %s", std::strerror(errno));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Enviados %zd bytes (cmd=%u, val=%u)", written, command, number);
    }
  }

  void cmd_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Mensaje recibido con menos de 2 elementos");
      return;
    }

    // IMPORTANTE: en tu Python hacías motor_der, motor_izq = msg.data
    // por eso en C++ replicamos el mismo orden:
    float motor_der = msg->data[0];
    float motor_izq = msg->data[1];

    RCLCPP_INFO(this->get_logger(), "Recibido: Izq: %.3f, Der: %.3f", motor_izq, motor_der);

    // multiplicabas por 100 y convertías a int:
    motors(static_cast<int>(motor_izq * 100.0f),
           static_cast<int>(motor_der * 100.0f));
  }

  // datos miembros
  std::string serial_port_;
  int fd_;
  std::mutex serial_mutex_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}