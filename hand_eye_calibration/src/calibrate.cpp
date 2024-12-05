#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>

#include "hand_eye_calibration/calibrate.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace hand_eye_calibration
{
using namespace std::chrono_literals;

Calibrate::Calibrate()
: rclcpp::Node("calibrate"), apriltag_ready_(false),
  px_curr_(0.0), py_curr_(0.0), px_saved_(0.0), py_saved_(0.0)
{
  rcl_interfaces::msg::ParameterDescriptor source_frame_des;
  rcl_interfaces::msg::ParameterDescriptor target_frame_des;

  source_frame_des.description = "Name of the source frame";
  target_frame_des.description = "Name of the target frame";

  declare_parameter<std::string>("source_frame", "camera_color_optical_frame", source_frame_des);
  declare_parameter<std::string>("target_frame", "tangram", target_frame_des);

  source_frame_ = get_parameter("source_frame").as_string();
  target_frame_ = get_parameter("target_frame").as_string();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = create_wall_timer(0.01s, std::bind(&Calibrate::timer_callback_, this));

  srv_save_ = create_service<std_srvs::srv::Trigger>(
    "april/save",
    std::bind(
      &Calibrate::srv_save_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  pub_apriltag_detect_ = create_publisher<tangram_msgs::msg::Point2D>("april/detect", 10);
  pub_apriltag_saved_ = create_publisher<tangram_msgs::msg::Point2D>("april/saved", 10);
}

void Calibrate::timer_callback_()
{
  if (apriltag_ready_) {
    tangram_msgs::msg::Point2D msg;
    msg.x = px_saved_;
    msg.y = py_saved_;

    pub_apriltag_saved_->publish(msg);
  } else {
    try {
      const auto tf = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);

      px_curr_ = tf.transform.translation.x;
      py_curr_ = tf.transform.translation.y;

      RCLCPP_DEBUG_STREAM(
        get_logger(), "Received transform from " << source_frame_ << " to " << target_frame_);
      RCLCPP_DEBUG_STREAM(get_logger(), "x: " << px_curr_ << ", y: " << py_curr_);

      tangram_msgs::msg::Point2D msg;
      msg.x = px_curr_;
      msg.y = py_curr_;

      pub_apriltag_detect_->publish(msg);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR_STREAM_ONCE(
        get_logger(),
        "Unable to get transform from "
          << source_frame_ << " to " << target_frame_
          << ": " << e.what()
      );
      return;
    }
  }
}

void Calibrate::srv_save_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;

  px_saved_ = px_curr_;
  py_saved_ = py_curr_;
  apriltag_ready_ = true;

  response->success = true;
}

} // namespace hand_eye_calibration

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hand_eye_calibration::Calibrate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
