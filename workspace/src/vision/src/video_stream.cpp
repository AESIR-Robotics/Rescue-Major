#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#if defined(ROS2_JAZZY)
    #include <cv_bridge/cv_bridge.hpp>
#elif defined(ROS2_HUMBLE)
    #include <cv_bridge/cv_bridge.h>
#else
    #error "ROS2 distro no soportada"
#endif

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "vision/srv/command.hpp"

using namespace std;

class VideoStreamPublisher : public rclcpp::Node {
public:
    VideoStreamPublisher()
    : Node("video_stream_publisher") {

        this->declare_parameter<string>("device", "jetson");
        this->declare_parameter<vector<long int>>("camera_devices_index", {0});
        this->declare_parameter<vector<string>>("capture_formats_", {"MJPG", "MJPG", "YUYV"});
        this->declare_parameter<std::vector<int64_t>>("camera_widths", {1920});
        this->declare_parameter<std::vector<int64_t>>("camera_heights", {1080});
        this->declare_parameter<std::vector<int64_t>>("camera_fps", {30});
        this->declare_parameter<int>("thermal_camera_index", -1);
        this->declare_parameter<bool>("thermal_enable", false);

        vector<long int> cam_indices;

        this->get_parameter("device", device_);
        this->get_parameter("camera_devices_index", cam_indices);
        this->get_parameter("capture_formats_", capture_formats_);
        this->get_parameter("camera_widths", camera_widths_);
        this->get_parameter("camera_heights", camera_heights_);
        this->get_parameter("camera_fps", camera_fps_);
        this->get_parameter("thermal_camera_index", thermal_camera_index_);
        this->get_parameter("thermal_enable", thermal_enabled_);

        this->declare_parameter<std::string>("pipeline_templates.MJPG", "v4l2src device=/dev/video{dev} ! image/jpeg, width={w}, height={h}, framerate={fps}/1 !  jpegparse ! nvv4l2decoder mjpeg=1 ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw,  format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false max-buffer=1");
        this->declare_parameter<std::string>("pipeline_templates.CSI", "nvarguscamerasrc sensor-id={dev} ! video/x-raw(memory:NVMM), width={w}, height={h}, format=NV12, framerate={fps}/1 ! nvvidconv ! video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");
        this->declare_parameter<std::string>("pipeline_templates.H264", "v4l2src device=/dev/video{dev} ! video/x-h264, width={w}, height={h}, framerate={fps}/1 ! h264parse ! nvv4l2decoder ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");
        this->declare_parameter<std::string>("pipeline_templates.YUYV", "v4l2src device=/dev/video{dev} ! video/x-raw, format=YUYV, width={w}, height={h}, framerate={fps}/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");
        
        camera_devices_.assign(cam_indices.begin(), cam_indices.end());

        // Publisher image vision sensors
        pub_sensors_ = this->create_publisher<sensor_msgs::msg::Image>("cam_sensors/image_raw", 10);
        num_cam_s_ = -1; 

        // Publisher image vision thermal
        pub_thermal_ = this->create_publisher<sensor_msgs::msg::Image>("cam_thermal/image_raw", 10);

        // Subscription commands
        command_service_ = this->create_service<vision::srv::Command>(
            "command_vision_videoStream",
            bind(&VideoStreamPublisher::command_callback, this, placeholders::_1, placeholders::_2)
        );

        // Solo agregar la térmica si el índice es válido (diferente de -1)
        if(thermal_camera_index_ != -1) {
            camera_devices_.push_back(thermal_camera_index_);
            cout << "Thermal camera added at index: " << thermal_camera_index_ << endl;
        }

        for( int i = 0; i < static_cast<int>(camera_devices_.size()); i++){
            cout << "Camera device index in vector pos " << i << " : /dev/video" << camera_devices_[i] << endl;
        }

        for (size_t i = 0; i < camera_devices_.size(); i++) {

            bool is_thermal = (camera_devices_[i] == thermal_camera_index_) && (thermal_camera_index_ != -1);

            if(!is_thermal){
                string topic_name = "cam" + to_string(i) + "/image_raw";
                auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
                publishers_.push_back(pub);
            }

            string dev = to_string(camera_devices_[i]);
            string w = to_string(camera_widths_[i]);
            string h = to_string(camera_heights_[i]);
            string fps = to_string(camera_fps_[i]);
            string format = capture_formats_[i];

            cv::VideoCapture cap;
            string pipeline = "";

            if(device_ == "jetson"){

                string template_key = "pipeline_templates." + format;
                string raw_template = this->get_parameter(template_key).as_string();

                if (raw_template.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Template for format '%s' is missing in YAML!", format.c_str());
                    continue;
                }

                pipeline = format_pipeline(raw_template, dev, w, h, fps);

                cout << "Opening Jetson pipeline for cam " << i << ":\n" << pipeline << endl;
                cap.open(pipeline, cv::CAP_GSTREAMER);
            }
            else if(device_ == "generic" || device_ == "raspberry"){
                // Configuración para PC Genérica
                cout << "Opening generic /dev/video" << camera_devices_[i] << endl;
                cap.open(camera_devices_[i], cv::CAP_V4L2);
                
                if(cap.isOpened()){
                    cap.set(cv::CAP_PROP_FRAME_WIDTH, stoi(w));
                    cap.set(cv::CAP_PROP_FRAME_HEIGHT, stoi(h));
                    cap.set(cv::CAP_PROP_FPS, stoi(fps));
                }
            }
            else{
                // Configuración para Laptop / PC Genérica
                cout << "No device specified for /dev/video" << camera_devices_[i] << endl;
                return;
            }

            if (!cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open camera index %d", camera_devices_[i]);
            } else {
                RCLCPP_INFO(this->get_logger(), "Camera index %d opened successfully", camera_devices_[i]);
            }
            cameras_.push_back(cap);
        }

        timer_1_ = this->create_wall_timer(
            chrono::milliseconds(33),
            bind(&VideoStreamPublisher::cameras_callback, this)
        );
        
        // Initial timer of thermal cam if it exist
        if(thermal_camera_index_ != -1){
            timer_2_ = this->create_wall_timer(
                chrono::milliseconds(33),
                bind(&VideoStreamPublisher::thermal_callback, this)
            );
        }
    }

private:

    string format_pipeline(std::string templ, const std::string& dev, const std::string& w, const std::string& h, const std::string& fps) {
        auto replace_all = [](std::string& str, const std::string& from, const std::string& to) {
            size_t start_pos = 0;
            while((start_pos = str.find(from, start_pos)) != std::string::npos) {
                str.replace(start_pos, from.length(), to);
                start_pos += to.length();
            }
        };
        
        replace_all(templ, "{dev}", dev);
        replace_all(templ, "{w}", w);
        replace_all(templ, "{h}", h);
        replace_all(templ, "{fps}", fps);
        
        return templ;
    }
    // --- CALLBACK DE VIDEO  ---
    void cameras_callback() {
        // Calcular el límite correcto para iterar
        size_t limit_index = cameras_.size();
        
        // Si tenemos cámara térmica configurada, la última del array es la térmica.
        // Restamos 1 al límite SOLO si existe cámara térmica.
        if (thermal_camera_index_ != -1 && !cameras_.empty()) {
            limit_index = cameras_.size() - 1;
        }

        for (size_t i = 0; i < limit_index; ++i) {

            if (!cameras_[i].isOpened()) continue;

            cv::Mat frame;
            cameras_[i] >> frame;
            
            if (frame.empty()) {
                // RCLCPP_WARN_ONCE(this->get_logger(), "Empty frame on camera %ld", i);
                continue;
            }

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            
            if(static_cast<int>(i) == num_cam_s_) {
                pub_sensors_->publish(*msg);
            }
            else {
                // Verificar que exista el publisher correspondiente
                if (i < publishers_.size()) {
                    publishers_[i]->publish(*msg);
                }
            }
        }
    }

    void thermal_callback() {
        if(!thermal_enabled_ || thermal_camera_index_ == -1 || cameras_.empty()) return;
        
        cv::VideoCapture& cap = cameras_.back();
        
        if (!cap.isOpened()) return;

        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_thermal_->publish(*msg);
    }

    void command_callback(const shared_ptr<vision::srv::Command::Request> request,
    shared_ptr<vision::srv::Command::Response> response){
        string cmd = request->data; 
        RCLCPP_INFO(this->get_logger(), "Service received: %s", cmd.c_str());

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
        else if (cmd.find("vision:camera,") == 0) {
            try {
                size_t comma_pos = cmd.find(',');
                int idx = stoi(cmd.substr(comma_pos + 1));
                
                // Validación de índice
                if (idx >= 0 && idx < static_cast<int>(cameras_.size())) {
                    // Toggle: si la cámara ya está activa para sensors, desactivarla
                    if (num_cam_s_ == idx) {
                        num_cam_s_ = -1;  // Ninguna cámara va a sensors
                        response->success = true;
                        response->message = "Camera " + to_string(idx) + " returned to raw (sensors disabled)";
                        RCLCPP_INFO(this->get_logger(), "Camera %d toggled OFF from sensors", idx);
                    } else {
                        // Activar esta cámara para sensors
                        num_cam_s_ = idx;
                        response->success = true;
                        response->message = "Camera " + to_string(idx) + " now publishing to sensors";
                        RCLCPP_INFO(this->get_logger(), "Camera %d toggled ON for sensors", idx);
                    }
                } else {
                    response->success = false;
                    response->message = "Camera index out of range (0-" + to_string(cameras_.size() - 1) + ")";
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
    string device_;
    bool thermal_enabled_;
    int thermal_camera_index_;
    vector<int> camera_devices_;
    vector<std::string> capture_formats_;
    vector<int64_t> camera_widths_;
    vector<int64_t> camera_heights_;
    vector<int64_t> camera_fps_;

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