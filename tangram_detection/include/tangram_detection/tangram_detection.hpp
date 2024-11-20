#ifndef TANGRAM_DETECTION__TANGRAM_DETECTION_HPP_
#define TANGRAM_DETECTION__TANGRAM_DETECTION_HPP_

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

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_tangram_poses_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  std::string image_dir_;

  cv::Mat source_img_;
  cv::Mat target_img_;

  void timer_callback_();
};
}  // namespace tangram_detection
#endif  // TANGRAM_DETECTION__TANGRAM_DETECTION_HPP_
