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
#include "vision/srv/command.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr int  kNoCamera       = -1;
static constexpr int  kTimerRgbMs     = 33;   // ~30 Hz
static constexpr int  kTimerThermalMs = 33;
static constexpr char kTopicSensors[] = "cam_sensors/image_raw";
static constexpr char kTopicThermal[] = "cam_thermal/image_raw";
static constexpr char kServiceName[]  = "command_vision_videoStream";

class VideoStreamPublisher : public rclcpp::Node 
{
public:
    explicit VideoStreamPublisher()
    : Node("video_stream_publisher")
    {

        // ── Parameter declaration ─────────────────────────────────────────────────

        // Device type: "jetson" | "generic" | "raspberry"
        this->declare_parameter<std::string>("device",                          "generic");
        this->declare_parameter<std::vector<long int>>("cameras_devices_index", {0, 2});
        this->declare_parameter<std::vector<std::string>>("cameras_formats_",   {"MJPG", "MJPG", "YUYV"});
        this->declare_parameter<std::vector<int64_t>>("cameras_widths",         {1920, 1920});
        this->declare_parameter<std::vector<int64_t>>("cameras_heights",        {1080, 1080});
        this->declare_parameter<std::vector<int64_t>>("cameras_fps",            {30, 30});

        // Thermal camera (-1 = disabled)
        this->declare_parameter<int>("thermal_camera_index", kNoCamera);
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

        load_parameters();
        init_publishers_to_services();
        init_cameras();
        init_timers();
    }

    ~VideoStreamPublisher()
    {
        release_cameras();
    }

private:

    // ── Load parameters  ─────────────────────────────────────────────────
    void load_parameters()
    {
        get_parameter("device",                device_);
        get_parameter("cameras_formats_",      capture_formats_);
        get_parameter("cameras_widths",        camera_widths_);
        get_parameter("cameras_heights",       camera_heights_);
        get_parameter("cameras_fps",           camera_fps_);
        get_parameter("thermal_camera_index",  thermal_camera_index_);
        get_parameter("thermal_enable",        thermal_enabled_);
 
        // Convert long int → int for camera device indices
        std::vector<long int> raw_indices;
        get_parameter("cameras_devices_index", raw_indices);
        camera_devices_.assign(raw_indices.begin(), raw_indices.end());
 
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
 
        if (device_ == "jetson") {
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

    // ── Timer initialisation ──────────────────────────────────────────────────
    void init_timers()
    {
        // RGB cameras at ~30 Hz
        timer_rgb_ = create_wall_timer(
            std::chrono::milliseconds(kTimerRgbMs),
            std::bind(&VideoStreamPublisher::cameras_callback, this));
 
        // Thermal camera timer (only if a thermal device is configured)
        if (thermal_camera_index_ != kNoCamera) {
            timer_thermal_ = create_wall_timer(
                std::chrono::milliseconds(kTimerThermalMs),
                std::bind(&VideoStreamPublisher::thermal_callback, this));
        }
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

    // ── Camera callbacks ───────────────────────────────────────────────────────
    void cameras_callback() {
        const size_t rgb_count = camera_count();
 
        for (size_t i = 0; i < rgb_count; ++i) {
            if (!cameras_[i].isOpened()) continue;
 
            cv::Mat frame;
            cameras_[i] >> frame;
            if (frame.empty()) continue;

            // If this camera is selected as the sensors source, publish there;
            // otherwise publish on its dedicated topic.
            if (static_cast<int>(i) == sensors_cam_idx_) {
                publish_frame(frame, pub_sensors_);
            } else if (i < publishers_.size()) {
                publish_frame(frame, publishers_[i]);
            }
        }
    }

    void thermal_callback() {
        if (!thermal_enabled_ ||
            thermal_camera_index_ == kNoCamera ||
            cameras_.empty()) return;
 
        cv::VideoCapture& cap = cameras_.back();
        if (!cap.isOpened()) return;
 
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) return;
 
        publish_frame(frame, pub_thermal_);
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

    // Publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_sensors_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_thermal_;

    // Service & timers
    rclcpp::Service<vision::srv::Command>::SharedPtr command_service_;
    rclcpp::TimerBase::SharedPtr                     timer_rgb_;
    rclcpp::TimerBase::SharedPtr                     timer_thermal_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}