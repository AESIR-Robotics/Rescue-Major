#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "vision/srv/command.hpp"

using namespace std;

class VideoStreamPublisher : public rclcpp::Node {
public:
    VideoStreamPublisher()
    : Node("video_stream_publisher") {
        this->declare_parameter<bool>("enable_jetson",false);
        this->declare_parameter<vector<long int>>("camera_devices_index",{0}); // add index camera in linux
        this->declare_parameter<int>("thermal_camera_index",3); // add index camera in linux
        this->declare_parameter<bool>("thermal_enable", true);

        vector<long int> cam_indices;

        this->get_parameter("enable_jetson", enable_jetson_);
        this->get_parameter("camera_devices_index", cam_indices);
        this->get_parameter("thermal_camera_index",thermal_camera_index_);
        this->get_parameter("thermal_enable", thermal_enabled_);
        camera_devices_.assign(cam_indices.begin(), cam_indices.end());

        // Publisher image vision sensors
        pub_sensors_ = this->create_publisher<sensor_msgs::msg::Image>("cam_sensors/image_raw", 10);
        num_cam_s_ = -1; // index of camera to send images to vision sensors node

        // Publisher image visio thermal
        pub_thermal_ = this->create_publisher<sensor_msgs::msg::Image>("cam_thermal/image_raw", 10);

        // Subcription commands
        command_service_ = this->create_service<vision::srv::Command>(
            "command_vision_videoStream",
            bind(&VideoStreamPublisher::command_callback, this, placeholders::_1, placeholders::_2)
        );
        camera_devices_.push_back(thermal_camera_index_);

        for (size_t i = 0; i < camera_devices_.size(); i++) {
            if(i != camera_devices_.size() - 1){
                string topic_name = "cam" + 
                to_string(i) + "/image_raw";
                auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
                publishers_.push_back(pub);
            }

            cv::VideoCapture cap;
            if(enable_jetson_){
                // NVIDIA accelerated (Jetson) MJPEG
                if(i != 0){
                    // Pipeline for secundary cameras
                    WIDTH=1280, HEIGHT=720, FPS=30;
                }
                else {
                    // Pipeline for main camera
                    WIDTH=1920, HEIGHT=1080, FPS=30;
                }
                string pipeline =
                    "v4l2src device=/dev/video" + to_string(camera_devices_[i]) + " ! "
                    "image/jpeg, width=" + to_string(WIDTH) + ", height=" + to_string(HEIGHT) + ", framerate=" + to_string(FPS) + "/1 !"
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
                cap.set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT);
                cap.set(cv::CAP_PROP_FPS, FPS);
                cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

                if (!cap.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open pipeline camera %zu", i);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Camera %zu pipeline opened successfully", i);
                }
            }

            // Note: this configuration depend most of hardware, type of camera and device where it's runnnig 
            if (!cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open camera %zu", i);
                continue;
            }
            cameras_.push_back(cap);
        }

        timer_1_ = this->create_wall_timer(
            chrono::milliseconds(33),
            bind(&VideoStreamPublisher::cameras_callback, this)
        );

        timer_2_ = this->create_wall_timer(
            chrono::milliseconds(33),
            bind(&VideoStreamPublisher::thermal_callback, this)
        );
    }

private:
    //Call back video
    void cameras_callback() {
        for (size_t i = 0; i < cameras_.size()-1; ++i) {
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

    // Call back video from thermal camera
    void thermal_callback() {
        if(thermal_enabled_ == false) return;
        cv::Mat frame;
        cameras_.back() >> frame;
        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        pub_thermal_->publish(*msg);
    }

    // Call back for commands
    void command_callback(const shared_ptr<vision::srv::Command::Request> request,
    shared_ptr<vision::srv::Command::Response> response){
        /*
         * Espera mensajes en formato
         * vision:<type>,<data>
         * Ejemplos: "vision:camera,0" "vision:thermal,on"
         */

        string cmd = request->data; // The string command
        RCLCPP_INFO(this->get_logger(), "Service received: %s", cmd.c_str());

        // Thermal camera
        if (cmd == "vision:thermal,on") {
            thermal_enabled_ = true;
            response->success = true;
            response->message = "Thermal enabled";
        } 
        else if (cmd == "vision:thermal,off") {
            thermal_enabled_ = false;
            response->success = true;
            response->message = "Thermal disabled";
        } 
        // Else cameras
        else if (cmd.find("vision:camera,") == 0) {
            try {
                size_t comma_pos = cmd.find(',');
                int idx = stoi(cmd.substr(comma_pos + 1));

                if (idx >= -11 && idx < static_cast<int>(cameras_.size())) {
                    num_cam_s_ = idx;
                    response->success = true;
                    response->message = "Active camera set to index " + to_string(idx);
                } else {
                    response->success = false;
                    response->message = "Camera index out of range";
                }
            } catch (...) {
                response->success = false;
                response->message = "Invalid camera index format";
            }
        } 
        else {
            response->success = false;
            response->message = "Unknown command: " + cmd;
        }
    }

private:    
    int num_cam_s_;
    vector<int> camera_devices_;
    bool enable_jetson_;
    bool thermal_enabled_;
    int thermal_camera_index_;
    int WIDTH, HEIGHT, FPS;

    vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    vector<cv::VideoCapture> cameras_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_sensors_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_thermal_;
    rclcpp::Service<vision::srv::Command>::SharedPtr command_service_;
    rclcpp::TimerBase::SharedPtr timer_1_;
    rclcpp::TimerBase::SharedPtr timer_2_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}
