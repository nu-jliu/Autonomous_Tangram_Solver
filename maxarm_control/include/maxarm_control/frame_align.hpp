#ifndef MAXARM_CONTROL__FRAME_ALIGN_HPP___
#define MAXARM_CONTROL__FRAME_ALIGN_HPP___

#include <armadillo>

#include <rclcpp/rclcpp.hpp>

#include "tangram_msgs/msg/tangram_poses.hpp"
#include "tangram_msgs/msg/point2_d.hpp"

namespace maxarm_control
{
class FrameAlign : public rclcpp::Node
{
public:
  FrameAlign();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<tangram_msgs::msg::TangramPoses>::SharedPtr sub_pieces_pose_cam_;
  rclcpp::Subscription<tangram_msgs::msg::Point2D>::SharedPtr sub_apriltag_saved_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_pieces_pose_robot_;

  bool apriltag_ready_;
  arma::mat33 T_robot_tag_;
  arma::mat33 T_camera_tag_;
  arma::mat33 T_robot_camera_;

  void sub_pieces_pos_cam_callback_(const tangram_msgs::msg::TangramPoses::SharedPtr msg);

  void sub_apriltag_saved_callback_(const tangram_msgs::msg::Point2D::SharedPtr msg);

  arma::mat33 trans_inv_(const arma::mat33 & T);

  void update_transform_();
};
}  // namespace maxarm_control
#endif  // MAXARM_CONTROL__FRAME_ALIGN_HPP___
