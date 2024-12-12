#ifndef PUZZLE_SOLVER__IMAGE_PUBLISHER_HPP___
#define PUZZLE_SOLVER__IMAGE_PUBLISHER_HPP___

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace puzzle_solver
{
class TangramPublisher : public rclcpp::Node
{
public:
  /// \brief The constructor of the tangram_publisher node
  TangramPublisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_inferenced_;

  cv::Mat image_;
  bool image_ready_;
  std::string image_full_path_;

  /// \brief Timer callback of the tangram publisher
  void timer_callback_();
};
}  /// namespace puzzle_solver
#endif  /// PUZZLE_SOLVER__IMAGE_PUBLISHER_HPP___
