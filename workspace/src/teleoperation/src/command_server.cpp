#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <sstream>
#include <signal.h>
#include <chrono>

typedef websocketpp::server<websocketpp::config::asio> server;

// Forward declaration

class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("command_server_pub") {
        RCLCPP_INFO(this->get_logger(), "Starting publisher node");
        vision_pub_ = this->create_publisher<std_msgs::msg::String>("commands_for_vision", 10);
        dc_motors_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("commands_for_dc_motors", 10);
    }

    void publish(const std::string& data) {
        size_t colon_pos = data.find(':');
        if (colon_pos == std::string::npos) return;

        std::string topic = data.substr(0, colon_pos);
        std::string payload = data.substr(colon_pos + 1);

        if (topic == "vision") {
            auto msg = std_msgs::msg::String();
            msg.data = payload;
            vision_pub_->publish(msg);
        } else if (topic == "dc_motors") {
            auto msg = std_msgs::msg::Float32MultiArray();
            
            size_t comma_pos = payload.find(',');
            if (comma_pos != std::string::npos) {
                float x = std::stof(payload.substr(0, comma_pos));
                float y = std::stof(payload.substr(comma_pos + 1));
                
                RCLCPP_INFO(this->get_logger(), "velocities: x=%f, y=%f", x, y);
                
                float right_motor = y + x;
                float left_motor = y - x;
                
                msg.data = {right_motor, left_motor};
                dc_motors_pub_->publish(msg);
            }
        }
        
        //RCLCPP_INFO(this->get_logger(), "Published: %s", data.c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dc_motors_pub_;
};

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("command_server_sub"), flag_(false) {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "send_to_client", 10,
            std::bind(&Subscriber::listener_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscriber initialized");
    }

    bool has_message() const { return flag_; }
    
    std::string get_message() {
        if (flag_) {
            flag_ = false;
            return info_;
        }
        return "";
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received message at listener: %s", msg->data.c_str());
        flag_ = true;
        info_ = msg->data;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::atomic<bool> flag_;
    std::string info_;
};

class CommandServer {
public:
    CommandServer(std::shared_ptr<Publisher> pub, std::shared_ptr<Subscriber> sub)
        : publisher_(pub), subscriber_(sub), stop_server_(false) {
        
        server_.set_access_channels(websocketpp::log::alevel::all);
        server_.clear_access_channels(websocketpp::log::alevel::frame_payload);
        
        server_.init_asio();
        server_.set_reuse_addr(true);  // Enable address reuse
        server_.set_message_handler(std::bind(&CommandServer::on_message, this, 
                                            std::placeholders::_1, std::placeholders::_2));
        server_.set_open_handler(std::bind(&CommandServer::on_open, this, 
                                         std::placeholders::_1));
        server_.set_close_handler(std::bind(&CommandServer::on_close, this, 
                                          std::placeholders::_1));
    }

    void start() {
        try {
            server_.listen(8082);
            server_.start_accept();
            
            // Start send loop in separate thread
            send_thread_ = std::thread(&CommandServer::send_loop, this);
            
            RCLCPP_INFO(rclcpp::get_logger("CommandServer"), "Server started on 0.0.0.0:8082");
            server_.run();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("CommandServer"), "Server error: %s", e.what());
            stop();
        }
    }

    void stop() {
        RCLCPP_INFO(rclcpp::get_logger("CommandServer"), "Stopping server...");
        stop_server_ = true;
        
        try {
            server_.stop_listening();
            server_.stop();
            
            if (send_thread_.joinable()) {
                send_thread_.join();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("CommandServer"), "Error stopping server: %s", e.what());
        }
    }

private:
    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        std::string message = msg->get_payload();
        RCLCPP_INFO(rclcpp::get_logger("CommandServer"), "Received message: %s", message.c_str());
        
        if (!message.empty()) {
            publisher_->publish(message);
        }
    }

    void on_open(websocketpp::connection_hdl hdl) {
        RCLCPP_INFO(rclcpp::get_logger("CommandServer"), "Client connected");
        connection_ = hdl;
    }

    void on_close(websocketpp::connection_hdl hdl) {
        RCLCPP_INFO(rclcpp::get_logger("CommandServer"), "Client disconnected");
    }

    void send_loop() {
        while (!stop_server_) {
            if (subscriber_->has_message()) {
                std::string message = subscriber_->get_message();
                if (!message.empty() && !connection_.expired()) {
                    try {
                        server_.get_alog().write(websocketpp::log::alevel::app, 
                                               "Sending: " + message);
                        server_.send(connection_, message, websocketpp::frame::opcode::text);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("CommandServer"), 
                                   "Send error: %s", e.what());
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    server server_;
    std::shared_ptr<Publisher> publisher_;
    std::shared_ptr<Subscriber> subscriber_;
    std::atomic<bool> stop_server_;
    std::thread send_thread_;
    websocketpp::connection_hdl connection_;
};

// Global pointer to server for signal handler
CommandServer* g_server = nullptr;

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("SignalHandler"), "Interrupt signal (%d) received.", signum);
    if (g_server) {
        g_server->stop();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    auto pub = std::make_shared<Publisher>();
    auto sub = std::make_shared<Subscriber>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pub);
    executor.add_node(sub);
    
    CommandServer server(pub, sub);
    g_server = &server;  // Set global pointer for signal handler
    
    // Start ROS spinning in separate thread
    std::thread ros_thread([&executor]() {
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // Start server (blocking)
    server.start();
    
    // Cleanup
    g_server = nullptr;
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}