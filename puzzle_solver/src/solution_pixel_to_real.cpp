#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "puzzle_solver/solution_pixel_to_real.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "tangram_msgs/msg/tangram_pose.hpp"

namespace puzzle_solver
{
SolutionPixelToReal::SolutionPixelToReal()
: rclcpp::Node("solution_pixel_to_real")
{
  rcl_interfaces::msg::ParameterDescriptor scale_des;
  rcl_interfaces::msg::ParameterDescriptor offset_des;

  scale_des.description = "Scale from the pixel to real";
  offset_des.description = "Offset of the workspace";

  declare_parameter<double>("scale", 0.00085, scale_des);
  declare_parameter<double>("offset", -0.45, offset_des);

  scale_ = get_parameter("scale").as_double();
  offset_ = get_parameter("offset").as_double();

  sub_tangram_pieces_pixel_ = create_subscription<tangram_msgs::msg::TangramPieces>(
    "place/pixel",
    10,
    std::bind(
      &SolutionPixelToReal::sub_tangram_pieces_pixel_callback_,
      this,
      std::placeholders::_1
    )
  );

  pub_tangram_pieces_real_ = create_publisher<tangram_msgs::msg::TangramPoses>("place/robot", 10);
}

void SolutionPixelToReal::sub_tangram_pieces_pixel_callback_(
  tangram_msgs::msg::TangramPieces::SharedPtr msg
)
{
  tangram_msgs::msg::TangramPoses poses_msg;

  for (const auto & p : msg->pieces) {
    tangram_msgs::msg::TangramPose pose;

    const int x_pixel = p.location.x;
    const int y_pixel = p.location.y;

    pose.location.x = static_cast<double>(x_pixel * scale_);
    pose.location.y = static_cast<double>(y_pixel * scale_ + offset_);

    pose.uuid = p.uuid;
    pose.type = p.type;

    poses_msg.poses.push_back(pose);
  }

  pub_tangram_pieces_real_->publish(poses_msg);
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<puzzle_solver::SolutionPixelToReal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
