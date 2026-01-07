#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <string>

using std::placeholders::_1;
using namespace std;

class VideoStreamPublisher : public rclcpp::Node {
public:
    VideoStreamPublisher()
    : Node("video_stream_publisher") {
        bool enable_jetson = false;
        camera_devices_ = {0}; // add more device indices here
        for (size_t i = 0; i < camera_devices_.size(); i++) {
            std::string topic_name = "cam" + std::to_string(i) + "/image_raw";
            auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
            publishers_.push_back(pub);

            cv::VideoCapture cap;
            if(enable_jetson){
                // NVIDIA accelerated (Jetson) H.264
                string pipeline = "v4l2src device=/dev/video" + to_string(camera_devices_[i]) +
                "! video/x-h264, width=1920, height=1080, framerate=30/1 \
                    ! h264parse \
                    ! nvv4l2decoder \
                    ! nvvidconv \
                        compute-hw=GPU \
                        nvbuf-memory-type=nvbuf-mem-cuda-device \
                        ! video/x-raw, format=BGRx \
                    ! videoconvert \
                    ! video/x-raw, format=BGR \
                    ! appsink";
                cap.open(pipeline, cv::CAP_GSTREAMER);
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
            std::chrono::milliseconds(20),
            std::bind(&VideoStreamPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        for (size_t i = 0; i < cameras_.size(); ++i) {
            cv::Mat frame;
            cameras_[i] >> frame;
            if (frame.empty()) continue;

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publishers_[i]->publish(*msg);
        }
    }

    using publish = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;

    std::vector<int> camera_devices_;
    std::vector<publish> publishers_;
    std::vector<cv::VideoCapture> cameras_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}