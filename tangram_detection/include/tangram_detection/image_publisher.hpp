#ifndef TANGRAM_DETECTION__IMAGE_PUBLISHER_HPP___
#define TANGRAM_DETECTION__IMAGE_PUBLISHER_HPP___

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace tangram_detection
{
class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  cv::Mat image_;
  bool image_ready_;
  std::string image_full_path_;

  void timer_callback_();
};
}  // namespace tangram_detection
#endif  // TANGRAM_DETECTION__IMAGE_PUBLISHER_HPP___
