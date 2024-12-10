#ifndef PUZZLE_SOLVER__WEB_CAMERA_HPP___
#define PUZZLE_SOLVER__WEB_CAMERA_HPP___

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

namespace puzzle_solver
{
class WebCamera : public rclcpp::Node
{
public:
  WebCamera();
  virtual ~WebCamera();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_webcam_image_;

  std::string camera_name_;
  std::string device_;

  cv::VideoCapture cap_;

  void timer_callback();
};
} /// namespace puzzle_solver
#endif /// PUZZLE_SOLVER__WEB_CAMERA_HPP___
