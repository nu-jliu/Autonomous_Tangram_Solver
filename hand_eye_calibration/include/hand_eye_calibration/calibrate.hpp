#ifndef HAND_EYE_CALIBRATION__CALIBRATE_HPP___
#define HAND_EYE_CALIBRATION__CALIBRATE_HPP___

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <std_srvs/srv/trigger.hpp>

#include "tangram_msgs/msg/point2_d.hpp"

namespace hand_eye_calibration
{
class Calibrate : public rclcpp::Node
{
public:
  Calibrate();

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;

  rclcpp::Publisher<tangram_msgs::msg::Point2D>::SharedPtr pub_apriltag_detect_;
  rclcpp::Publisher<tangram_msgs::msg::Point2D>::SharedPtr pub_apriltag_saved_;

  bool apriltag_ready_;
  double px_curr_;
  double py_curr_;
  double px_saved_;
  double py_saved_;

  std::string source_frame_;
  std::string target_frame_;

  void timer_callback_();

  void srv_save_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );
};
}  // namespace hand_eye_calibration
#endif  // HAND_EYE_CALIBRATION__CALIBRATE_HPP___
