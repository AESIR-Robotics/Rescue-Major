#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std;

class VideoStreamPublisher : public rclcpp::Node {
public:
    VideoStreamPublisher()
    : Node("video_stream_publisher") {
        this->declare_parameter<bool>("enable_jetson",false);
        this->declare_parameter<std::vector<int>>("camera_devices",{0, 2, 4}); // add more device indices here
        std::vector<long int> camera_devices_long;
        this->get_parameter("enable_jetson", enable_jetson);
        this->get_parameter("camera_devices", camera_devices_long);
        camera_devices_.assign(camera_devices_long.begin(), camera_devices_long.end());

        // Publisher image vision sensors
        pub_sensors_ = this->create_publisher<sensor_msgs::msg::Image>("cam_raw/sensors", 10);
        num_cam_s_ = -1; // index of camera to send images to vision sensors node

        // Subcription commands
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "commands_vision",
            10,
            std::bind(&VideoStreamPublisher::command_callback, this, std::placeholders::_1)
        );

        int WIDTH=1920, HEIGHT=1080, FPS=30;

        for (size_t i = 0; i < camera_devices_.size(); i++) {
            std::string topic_name = "cam" + std::to_string(i) + "/image_raw";
            auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
            publishers_.push_back(pub);

            cv::VideoCapture cap;
            if(enable_jetson){
                // NVIDIA accelerated (Jetson) MJPEG

                if(i != 0){
                    // Pipeline for secundary cameras
                    WIDTH=1280, HEIGHT=720, FPS=30;
                }
                else {
                    // Pipeline for main camera
                    WIDTH=1920, HEIGHT=1080, FPS=30;
                }
                std::string pipeline =
                    "v4l2src device=/dev/video" + std::to_string(camera_devices_[i]) + " ! "
                    "image/jpeg, width=" + std::to_string(WIDTH) + ", height=" + std::to_string(HEIGHT) + ", framerate=" + std::to_string(FPS) + "/1 !"
                    "jpegparse ! " 
                    "nvv4l2decoder ! video/x-raw(memory:NVMM) !"               
                    "nvvidconv ! video/x-raw, format=BGRx ! "
                    "videoconvert ! video/x-raw, format=BGR ! " 
                    "appsink drop=true sync=false max-buffer=1";
                cap.open(pipeline, cv::CAP_GSTREAMER);

    
                if (!cap.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline for camera %zu", i);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Camera %zu pipeline opened successfully", i);
                }
            }
            else{
                // Optimatize for generic computer
                cap.open(camera_devices_[i], cv::CAP_V4L2);
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
                cap.set(cv::CAP_PROP_FPS, 30);
                cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
            }

            // Note: this configuration depend most of hardware, type of camera and device where it's runnnig 

            if (!cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open camera %zu", i);
                continue;
            }
            cameras_.push_back(cap);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&VideoStreamPublisher::timer_callback, this)
        );
    }

private:

    //Call back video
    void timer_callback() {
        for (size_t i = 0; i < cameras_.size(); ++i) {
            cv::Mat frame;

            cameras_[i] >> frame;
            if (frame.empty()) continue;

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            if(static_cast<int>(i) == num_cam_s_) {
                //camera select to sensors node 
                pub_sensors_->publish(*msg);
            }
            else {
                //camera to interface
                publishers_[i]->publish(*msg);
            }
        }
    }

    // Call back for commands
    void command_callback(const std_msgs::msg::String::SharedPtr msg){
        //RCLCPP_INFO(this->get_logger(), "Mensaje recibido: %s", msg->data.c_str());
        /*
         * Espera mensajes en formato
         * vision:<num_cam>
         * Ejemplo: "vision:num_cam,0"
         */

        const std::string prefix = "vision:num_cam,";
        // Check if the message starts with the expected prefix
        if (msg->data.rfind(prefix, 0) != 0) {
            RCLCPP_DEBUG(this->get_logger(), "Mensaje ignorado: no es para vision.");
            return;
        }

        // Extract the mode number
        std::string cam_str = msg->data.substr(prefix.length());
        int new_cam;

        try {
            new_cam = std::stoi(cam_str); // Convert string to integer
        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo convertir el modo '%s' a numero.", cam_str.c_str());
            return;
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "Valor de modo fuera de rango: %s", cam_str.c_str());
            return;
        }

        // Validate the mode range
        if (new_cam < 0 || new_cam > 3) {
            RCLCPP_WARN(this->get_logger(), "Modo %d fuera de rango (0-3).", new_cam);
            return;
        }

        // Update the class member variable
        this->num_cam_s_ = new_cam;
        RCLCPP_INFO(this->get_logger(), "Modo actualizado a %d", this->num_cam_s_);
    }

private:    
    int num_cam_s_;

    std::vector<int> camera_devices_;
    bool enable_jetson;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::vector<cv::VideoCapture> cameras_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_sensors_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}
