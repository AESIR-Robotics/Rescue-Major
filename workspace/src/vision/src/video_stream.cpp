#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#if defined(ROS2_JAZZY)
    #include <cv_bridge/cv_bridge.hpp>
#elif defined(ROS2_HUMBLE)
    #include <cv_bridge/cv_bridge.h>
#else
    #error "Unsupported ROS2 distribution"
#endif

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <memory>
#include <functional>
#include <filesystem>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include "vision/srv/command.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr int  kNoCamera       = -1;
static constexpr char kTopicSensors[] = "cam_sensors/image_raw";
static constexpr char kTopicThermal[] = "cam_thermal/image_raw";
static constexpr char kServiceName[]  = "command_vision_videoStream";

class VideoStreamPublisher : public rclcpp::Node 
{
public:
    explicit VideoStreamPublisher()
    : Node("video_stream_publisher")
    {
        declare_parameters();
        load_parameters();
        init_publishers_to_services();
        init_cameras();
        //init_timers();
    }

    ~VideoStreamPublisher()
    {
        running_ = false;
        for (auto& t : capture_threads_)
            if (t.joinable()) t.join();
        release_cameras();
    }

private:

    // ── Parameters declaration ─────────────────────────────────────────────────
    void declare_parameters()
    {
        // ── Parameters declaration ─────────────────────────────────────────────────

        // Device type: "jetson" | "generic" | "raspberry"
        this->declare_parameter<std::string>("device",                          "generic");
        this->declare_parameter<std::vector<std::string>>("cameras_devices",    {"0", "2"});
        this->declare_parameter<std::vector<std::string>>("cameras_formats",   {"MJPG", "MJPG", "YUYV"});
        this->declare_parameter<std::vector<int64_t>>("cameras_widths",         {1920, 1920});
        this->declare_parameter<std::vector<int64_t>>("cameras_heights",        {1080, 1080});
        this->declare_parameter<std::vector<int64_t>>("cameras_fps",            {30, 30});

        // Thermal camera (-1 = disabled)
        this->declare_parameter<std::string>("thermal_camera_device", std::to_string(kNoCamera));
        this->declare_parameter<bool>("thermal_enable",      false);

        std::vector<long int> cam_indices;

        // GStreamer pipeline templates (Jetson only)
        declare_parameter<std::string>("pipeline_templates.MJPG",
            "v4l2src device=/dev/video{dev} ! image/jpeg, width={w}, height={h}, framerate={fps}/1 ! "
            "jpegparse ! nvv4l2decoder mjpeg=1 ! video/x-raw(memory:NVMM) ! nvvidconv ! "
            "video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false max-buffer=1");

        declare_parameter<std::string>("pipeline_templates.CSI",
            "nvarguscamerasrc sensor-id={dev} ! "
            "video/x-raw(memory:NVMM), width={w}, height={h}, format=NV12, framerate={fps}/1 ! "
            "nvvidconv ! video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");

        declare_parameter<std::string>("pipeline_templates.H264",
            "v4l2src device=/dev/video{dev} ! video/x-h264, width={w}, height={h}, framerate={fps}/1 ! "
            "h264parse ! nvv4l2decoder ! video/x-raw(memory:NVMM) ! nvvidconv ! "
            "video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");

        declare_parameter<std::string>("pipeline_templates.YUYV",
            "v4l2src device=/dev/video{dev} ! video/x-raw, format=YUYV, width={w}, height={h}, "
            "framerate={fps}/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! "
            "video/x-raw, format=I420 ! appsink drop=true sync=false max-buffer=1");

        declare_parameter<std::string>("pipeline_templates.THERMAL_BGR","");
    }

    // ── Load parameters  ─────────────────────────────────────────────────
    void load_parameters()
    {
        get_parameter("device",                device_);
        get_parameter("cameras_formats",       capture_formats_);
        get_parameter("cameras_widths",        camera_widths_);
        get_parameter("cameras_heights",       camera_heights_);
        get_parameter("cameras_fps",           camera_fps_);
        get_parameter("thermal_enable",        thermal_enabled_);
 
        // Convert long int → int for camera device indices
        std::vector<std::string> raw_devices;
        get_parameter("cameras_devices", raw_devices);
        
        camera_devices_.clear();
        for (const auto& dev_str : raw_devices) {
            int resolved_idx = resolve_device(dev_str);
            if (resolved_idx != kNoCamera) {
                camera_devices_.push_back(resolved_idx);
            } else {
                RCLCPP_WARN(get_logger(), "Skipping unresolved camera: '%s'", dev_str.c_str());
            }
        }

        // Resolve thermal camera
        std::string thermal_str;
        get_parameter("thermal_camera_device", thermal_str);
        thermal_camera_index_ = resolve_device(thermal_str);
 
        // Thermal device is appended last so cameras_ slot indexing stays consistent
        if (thermal_camera_index_ != kNoCamera) {
            camera_devices_.push_back(thermal_camera_index_);
            RCLCPP_INFO(get_logger(),
                "Thermal camera scheduled at /dev/video%d", thermal_camera_index_);
        }
 
        for (size_t i = 0; i < camera_devices_.size(); ++i) {
            RCLCPP_INFO(get_logger(),
                "Camera slot %zu → /dev/video%d", i, camera_devices_[i]);
        }
    }

     // ── Helper: Device Resolution
    /**
     * Resolves a string identifier to a V4L2 device integer.
     * If the string is purely numeric (e.g., "0"), returns 0.
     * If it's a name (e.g., "HD WebCam"), it looks in sysfs to find the matching /dev/videoX. 
     */
    int resolve_device(const std::string& identifier)
    {
        // Check if the string is purely numeric (an index)
        if (identifier.find_first_not_of("0123456789") == std::string::npos || 
           (identifier.front() == '-' && identifier.find_first_not_of("0123456789", 1) == std::string::npos)) {
            try { return std::stoi(identifier); } 
            catch (...) { return kNoCamera; }
        }

        // Search by device name in /sys/class/video4linux
        namespace fs = std::filesystem;
        const std::string v4l_path = "/sys/class/video4linux/";
        
        if (!fs::exists(v4l_path)) {
            RCLCPP_WARN(get_logger(), "Sysfs path %s does not exist. Cannot resolve by name.", v4l_path.c_str());
            return kNoCamera;
        }

        std::vector<int> matching_indices;

        for (const auto& entry : fs::directory_iterator(v4l_path)) {
            std::string dir_name = entry.path().filename().string();
            
            // Directories look like "video0", "video1", etc.
            if (dir_name.find("video") == 0) {
                std::ifstream name_file(entry.path() / "name");
                if (name_file.is_open()) {
                    std::string device_name;
                    std::getline(name_file, device_name);
                    
                    // Case-sensitive substring match
                    if (device_name.find(identifier) != std::string::npos) {
                        std::string idx_str = dir_name.substr(5); // Extract "0" from "video0"
                        try {
                            RCLCPP_INFO(get_logger(), "Resolved name '%s' to /dev/video%s", identifier.c_str(), idx_str.c_str());
                            matching_indices.push_back(std::stoi(idx_str));
                        } catch (...) {}
                    }
                }
            }
        }

        // Return the lowest index (avoids metadata nodes)
        if (!matching_indices.empty()) {
            int best_idx = *std::min_element(matching_indices.begin(), matching_indices.end());
            RCLCPP_INFO(get_logger(), "Resolved name '%s' to /dev/video%d", identifier.c_str(), best_idx);
            return best_idx;
        }
        
        RCLCPP_ERROR(get_logger(), "Could not find any camera matching name: '%s'", identifier.c_str());
        return kNoCamera;
    }

        // ── Helper: number of RGB cameras ────────────────────────────────────────
    size_t camera_count() const
    {
        if (thermal_camera_index_ != kNoCamera && !camera_devices_.empty()) {
            return camera_devices_.size() - 1;
        }
        return camera_devices_.size();
    }

    // ── Helper: build and publish one frame ───────────────────────────────────
    void publish_frame(
        const cv::Mat& frame,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub)
    {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub->publish(*msg);
    }

    // ── Helper: GStreamer pipeline builder ────────────────────────────────────
    static std::string build_pipeline(
        std::string        tmpl,
        const std::string& dev,
        const std::string& w,
        const std::string& h,
        const std::string& fps)
    {
        auto replace_all = [](std::string& s,
                               const std::string& from,
                               const std::string& to)
        {
            for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size())
                s.replace(pos, from.size(), to);
        };

        replace_all(tmpl, "{dev}", dev);
        replace_all(tmpl, "{w}",   w);
        replace_all(tmpl, "{h}",   h);
        replace_all(tmpl, "{fps}", fps);
        return tmpl;
    }

    // ── Publishers / Service ──────────────────────────────────────────────────
    void init_publishers_to_services()
    {
        // "Sensors" re-route topic: any camera can be redirected here at runtime
        pub_sensors_ = create_publisher<sensor_msgs::msg::Image>(kTopicSensors, 10);

        // Individual per-camera topics (exclude the thermal slot)
        pub_thermal_ = create_publisher<sensor_msgs::msg::Image>(kTopicThermal, 10);

        const size_t rgb_count = camera_count();
 
        publishers_.reserve(rgb_count);
 
        for (size_t i = 0; i < rgb_count; ++i) {
            publishers_.push_back(
                create_publisher<sensor_msgs::msg::Image>(
                    "cam" + std::to_string(i) + "/image_raw", 10));
        }

        // Command service
        command_service_ = create_service<vision::srv::Command>(
            kServiceName,
            std::bind(&VideoStreamPublisher::handle_command_callback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2));
    }

    // ── Camera initialisation ─────────────────────────────────────────────────
    void init_cameras()
    {
        cameras_.reserve(camera_devices_.size());
 
        for (size_t idx = 0; idx < camera_devices_.size(); ++idx) {
            cv::VideoCapture cap = open_camera(idx);
 
            if (!cap.isOpened()) {
                RCLCPP_ERROR(get_logger(),
                    "Could not open camera /dev/video%d", camera_devices_[idx]);
            } else {
                RCLCPP_INFO(get_logger(),
                    "Camera /dev/video%d opened (slot %zu)", camera_devices_[idx], idx);
            }
 
            cameras_.emplace_back(std::move(cap));
        }

        start_capture_threads(); 
    }


    void start_capture_threads()
    {
        const size_t rgb_count = camera_count();

        // 1. Iniciar hilos para las cámaras RGB normales
        for (size_t i = 0; i < rgb_count; ++i) {
            capture_threads_.emplace_back(&VideoStreamPublisher::rgb_capture_loop, this, i);
        }

        // 2. Iniciar hilo exclusivo para la térmica (si existe)
        if (thermal_camera_index_ != kNoCamera) {
            size_t thermal_idx = cameras_.size() - 1;
            capture_threads_.emplace_back(&VideoStreamPublisher::thermal_capture_loop, this, thermal_idx);
        }
    }

    void rgb_capture_loop(size_t i)
    {
        constexpr int kMaxFailures = 10;
        int consecutive_failures = 0;

        while (running_) {

            // Reconnect loop 
            if (!cameras_[i].isOpened()) {
                RCLCPP_WARN(get_logger(),
                    "Cam RGB slot %zu (/dev/video%d) not open — attempting reconnect...",
                    i, camera_devices_[i]);

                std::vector<std::string> raw_devices;
                get_parameter("cameras_devices", raw_devices);
                if (i < raw_devices.size()) {
                    int new_idx = resolve_device(raw_devices[i]);
                    if (new_idx != kNoCamera) camera_devices_[i] = new_idx;
                }

                cv::VideoCapture new_cap = open_camera(i);
                if (new_cap.isOpened()) {
                    cameras_[i] = std::move(new_cap);
                    consecutive_failures = 0;
                    RCLCPP_INFO(get_logger(),
                        "Cam slot %zu (/dev/video%d) reconnected.", i, camera_devices_[i]);
                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                continue;
            }

            // Capture
            if (!cameras_[i].grab()) {
                consecutive_failures++;
                if (consecutive_failures >= kMaxFailures) {
                    RCLCPP_WARN(get_logger(),
                        "Cam slot %zu (/dev/video%d) — %d consecutive failures, releasing.",
                        i, camera_devices_[i], consecutive_failures);
                    cameras_[i].release();  // triggers reconnect loop next iteration
                    consecutive_failures = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                continue;
            }

            consecutive_failures = 0;

            cv::Mat frame;
            cameras_[i].retrieve(frame);
            if (frame.empty()) continue;

            // Publish
            const bool is_thermal = (thermal_camera_index_ != kNoCamera)
                                && (i == cameras_.size() - 1);

            if (is_thermal) {
                if (thermal_enabled_) publish_frame(frame, pub_thermal_);
            } else if (static_cast<int>(i) == sensors_cam_idx_) {
                publish_frame(frame, pub_sensors_);
            } else if (i < publishers_.size()) {
                publish_frame(frame, publishers_[i]);
            }
        }
    }

    void thermal_capture_loop(size_t i)
    {
        constexpr int kMaxFailures = 10;
        int consecutive_failures = 0;

        const auto frame_duration = std::chrono::milliseconds(850 / camera_fps_[i]); // Le baje un poquito que diera 9 fps aprox :)
        auto last_publish_time = std::chrono::steady_clock::now();

        while (running_) {

            // Reconnect loop 
            if (!cameras_[i].isOpened()) {
                RCLCPP_WARN(get_logger(),
                    "Cam Thermal slot %zu (/dev/video%d) not open — attempting reconnect...",
                    i, camera_devices_[i]);

                std::string thermal_str;
                get_parameter("thermal_camera_device", thermal_str);

                int new_idx = resolve_device(thermal_str);
                if (new_idx != kNoCamera) camera_devices_[i] = new_idx;

                cv::VideoCapture new_cap = open_camera(i);
                if (new_cap.isOpened()) {
                    cameras_[i] = std::move(new_cap);
                    consecutive_failures = 0;
                    RCLCPP_INFO(get_logger(),
                        "Cam slot %zu (/dev/video%d) reconnected.", i, camera_devices_[i]);
                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                continue;
            }

            // Capture
            if (!cameras_[i].grab()) {
                consecutive_failures++;
                if (consecutive_failures >= kMaxFailures) {
                    RCLCPP_WARN(get_logger(),
                        "Cam slot %zu (/dev/video%d) — %d consecutive failures, releasing.",
                        i, camera_devices_[i], consecutive_failures);
                    cameras_[i].release();  // triggers reconnect loop next iteration
                    consecutive_failures = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                continue;
            }

            consecutive_failures = 0;

            auto now = std::chrono::steady_clock::now();
            if (now - last_publish_time < frame_duration) {
                continue; 
            }
            last_publish_time = now;

            cv::Mat frame;
            cameras_[i].retrieve(frame);
            if (frame.empty()) continue;

            if (thermal_enabled_) {
                if (thermal_enabled_) publish_frame(frame, pub_thermal_);
            }
        }
    }
 
    /**
     * Open a single camera at slot @p idx.
     * Returns an un-opened VideoCapture on failure — caller must check isOpened().
     */
    cv::VideoCapture open_camera(size_t idx)
    {
        const std::string dev = std::to_string(camera_devices_[idx]);
        const std::string w   = std::to_string(camera_widths_[idx]);
        const std::string h   = std::to_string(camera_heights_[idx]);
        const std::string fps = std::to_string(camera_fps_[idx]);
        const std::string fmt = capture_formats_[idx];
 
        cv::VideoCapture cap;

        // For open the thermal cam
        if (fmt == "THERMAL_BGR") {
            RCLCPP_INFO(get_logger(), "Opening /dev/video%s via direct V4L2 (Bypassing GStreamer)", dev.c_str());
            cap.open(camera_devices_[idx], cv::CAP_V4L2);

            if (cap.isOpened()) {
                cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
                cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B','G','R','3'));
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 80);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 60);
                cap.set(cv::CAP_PROP_FPS, 9);
            }
        }
        else if (device_ == "jetson") {
            const std::string key      = "pipeline_templates." + fmt;
            const std::string raw_tmpl = get_parameter(key).as_string();
 
            if (raw_tmpl.empty()) {
                RCLCPP_ERROR(get_logger(),
                    "No GStreamer template for format '%s'", fmt.c_str());
                return cap; // un-opened
            }
 
            const std::string pipeline = build_pipeline(raw_tmpl, dev, w, h, fps);
            RCLCPP_INFO(get_logger(), "Jetson pipeline [cam%zu]:\n%s", idx, pipeline.c_str());
            cap.open(pipeline, cv::CAP_GSTREAMER);
        }
        else if (device_ == "generic" || device_ == "raspberry") {
            RCLCPP_INFO(get_logger(), "Opening generic /dev/video%s", dev.c_str());
            cap.open(camera_devices_[idx], cv::CAP_V4L2);
 
            if (cap.isOpened()) {
                cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                cap.set(cv::CAP_PROP_FRAME_WIDTH,  std::stoi(w));
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, std::stoi(h));
                cap.set(cv::CAP_PROP_FPS,          std::stoi(fps));
            }
        }
        else {
            RCLCPP_ERROR(get_logger(), "Unknown device type: '%s'", device_.c_str());
        }
 
        return cap;
    }

    // ── Service callback ──────────────────────────────────────────────────────
    void handle_command_callback(
    const std::shared_ptr<vision::srv::Command::Request> request,
    std::shared_ptr<vision::srv::Command::Response> response){
        std::string cmd = request->data; 
        RCLCPP_INFO(this->get_logger(), "Service received: %s", cmd.c_str());


        // -- handle thermal enable/disable --
        if (cmd == "vision:thermal,on") {
            thermal_enabled_  = true;
            response->success = true;
            response->message = "Thermal enabled";
        } 
        else if (cmd == "vision:thermal,off") {
            thermal_enabled_  = false;
            response->success = true;
            response->message = "Thermal disabled";
        } 

        // -- handle sensors topic camera toggle --
        else if (cmd.find("vision:camera,") == 0) {
            try {
                const int idx = std::stoi(cmd.substr(cmd.find(',') + 1));
                const int max = camera_count();

                if (idx < 0 || idx > max) {
                    response->success = false;
                    response->message = "Camera index out of range (0–" + std::to_string(max) + ")";
                    return;
                }

                if (sensors_cam_idx_ == idx) {
                    // Toggle off: camera returns to its dedicated topic
                    sensors_cam_idx_  = kNoCamera;
                    response->success = true;
                    response->message = "Camera " + std::to_string(idx) + " removed from sensors topic";
                    RCLCPP_INFO(get_logger(), "Camera %d → off sensors", idx);
                } else {
                    // Toggle on: redirect this camera to the sensors topic
                    sensors_cam_idx_  = idx;
                    response->success = true;
                    response->message = "Camera " + std::to_string(idx) + " now publishing to sensors";
                    RCLCPP_INFO(get_logger(), "Camera %d → on sensors", idx);
                }
            }
            catch (...) {
                response->success = false;
                response->message = "Invalid camera index format";
            }
        } 
        else {
            response->success = false;
            response->message = "Unknown command: " + cmd;
        }
    }

        // ── Camera cleanup ────────────────────────────────────────────────────────
    void release_cameras()
    {
        for (auto& cap : cameras_) {
            if (cap.isOpened()) {
                cap.release();
            }
        }
        cameras_.clear();
        RCLCPP_INFO(get_logger(), "All cameras released.");
    }

    // ── Member variables ──────────────────────────────────────────────────────

    // Configuration
    std::string device_;
    bool        thermal_enabled_{false};
    int         thermal_camera_index_{kNoCamera};
    int         sensors_cam_idx_{kNoCamera}; // Which RGB cam → sensors topic
    std::vector<int>         camera_devices_;
    std::vector<std::string> capture_formats_;
    std::vector<int64_t>     camera_widths_;
    std::vector<int64_t>     camera_heights_;
    std::vector<int64_t>     camera_fps_;

    // OpenCV capture handles 
    std::vector<cv::VideoCapture> cameras_;

    // Capture threads
    std::vector<std::thread>    capture_threads_;
    std::atomic<bool>           running_{true};

    // Publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_sensors_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_thermal_;

    // Services
    rclcpp::Service<vision::srv::Command>::SharedPtr command_service_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}