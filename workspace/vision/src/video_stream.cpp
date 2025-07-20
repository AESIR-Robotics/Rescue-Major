#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std;

class VideoStreamPublisher : public rclcpp::Node {
public:
    VideoStreamPublisher()
    : Node("video_stream_publisher") {
        camera_devices_ = {0, 1}; // add more device indices here
        for (size_t i = 0; i < camera_devices_.size(); i++) {
            std::string topic_name = "cam" + std::to_string(i) + "/image_raw";
            auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
            publishers_.push_back(pub);

            cv::VideoCapture cap(camera_devices_[i] * 2);
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

    std::vector<int> camera_devices_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::vector<cv::VideoCapture> cameras_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoStreamPublisher>());
    rclcpp::shutdown();
    return 0;
}