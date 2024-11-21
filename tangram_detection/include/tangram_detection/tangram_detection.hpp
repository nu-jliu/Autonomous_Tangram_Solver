#ifndef TANGRAM_DETECTION__TANGRAM_DETECTION_HPP___
#define TANGRAM_DETECTION__TANGRAM_DETECTION_HPP___

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include "tangram_msgs/msg/tangram_poses.hpp"
#include <sensor_msgs/msg/image.hpp>

namespace tangram_detection
{
class TangramDetection : public rclcpp::Node
{
public:
  TangramDetection();
  ~TangramDetection();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_tangram_poses_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_source_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_erode_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_dialate_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_opened_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_closed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_edges_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contours_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contours_approx_;

  bool image_ready_;
  cv::Mat source_img_;
  cv::Mat target_img_;

  void timer_callback_();

  void sub_tangram_image_callback_(sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace tangram_detection
#endif  // TANGRAM_DETECTION__TANGRAM_DETECTION_HPP_
