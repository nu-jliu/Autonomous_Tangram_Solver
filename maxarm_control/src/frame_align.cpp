#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "maxarm_control/frame_align.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "tangram_msgs/msg/tangram_pose.hpp"

namespace maxarm_control
{
FrameAlign::FrameAlign()
: rclcpp::Node("frame_align"), apriltag_ready_(false)
{
  rcl_interfaces::msg::ParameterDescriptor rt_r00_des;
  rcl_interfaces::msg::ParameterDescriptor rt_r01_des;
  rcl_interfaces::msg::ParameterDescriptor rt_r10_des;
  rcl_interfaces::msg::ParameterDescriptor rt_r11_des;
  rcl_interfaces::msg::ParameterDescriptor rt_px_des;
  rcl_interfaces::msg::ParameterDescriptor rt_py_des;
  rcl_interfaces::msg::ParameterDescriptor ct_r00_des;
  rcl_interfaces::msg::ParameterDescriptor ct_r01_des;
  rcl_interfaces::msg::ParameterDescriptor ct_r10_des;
  rcl_interfaces::msg::ParameterDescriptor ct_r11_des;
  rcl_interfaces::msg::ParameterDescriptor ct_px_des;
  rcl_interfaces::msg::ParameterDescriptor ct_py_des;

  rt_r00_des.description = "R00 component of the transformation matrix from robot to tag";
  rt_r01_des.description = "R01 component of the transformation matrix from robot to tag";
  rt_r10_des.description = "R10 component of the transformation matrix from robot to tag";
  rt_r11_des.description = "R11 component of the transformation matrix from robot to tag";
  rt_px_des.description = "X component of the robot to tag transformation";
  rt_py_des.description = "Y component of the robot to tag transformation";
  ct_r00_des.description = "R00 component of the transformation matrix from camera to tag";
  ct_r01_des.description = "R01 component of the transformation matrix from camera to tag";
  ct_r10_des.description = "R10 component of the transformation matrix from camera to tag";
  ct_r11_des.description = "R11 component of the transformation matrix from camera to tag";
  ct_px_des.description = "X component of the camera to tag transformation";
  ct_py_des.description = "Y component of the camera to tag transformation";

  declare_parameter<double>("robot_tag.r00", 0.0, rt_r00_des);
  declare_parameter<double>("robot_tag.r01", 0.0, rt_r01_des);
  declare_parameter<double>("robot_tag.r10", 0.0, rt_r10_des);
  declare_parameter<double>("robot_tag.r11", 0.0, rt_r11_des);
  declare_parameter<double>("robot_tag.px", 0.0, rt_px_des);
  declare_parameter<double>("robot_tag.py", 0.0, rt_py_des);
  declare_parameter<double>("camera_tag.r00", 0.0, rt_r00_des);
  declare_parameter<double>("camera_tag.r01", 0.0, rt_r01_des);
  declare_parameter<double>("camera_tag.r10", 0.0, rt_r10_des);
  declare_parameter<double>("camera_tag.r11", 0.0, rt_r11_des);
  declare_parameter<double>("camera_tag.px", 0.0, rt_px_des);
  declare_parameter<double>("camera_tag.py", 0.0, rt_py_des);

  const auto rt_r00 = get_parameter("robot_tag.r00").as_double();
  const auto rt_r01 = get_parameter("robot_tag.r01").as_double();
  const auto rt_r10 = get_parameter("robot_tag.r10").as_double();
  const auto rt_r11 = get_parameter("robot_tag.r11").as_double();
  const auto rt_px = get_parameter("robot_tag.px").as_double();
  const auto rt_py = get_parameter("robot_tag.py").as_double();
  const auto ct_r00 = get_parameter("camera_tag.r00").as_double();
  const auto ct_r01 = get_parameter("camera_tag.r01").as_double();
  const auto ct_r10 = get_parameter("camera_tag.r10").as_double();
  const auto ct_r11 = get_parameter("camera_tag.r11").as_double();
  const auto ct_px = get_parameter("camera_tag.px").as_double();
  const auto ct_py = get_parameter("camera_tag.py").as_double();

  T_robot_tag_ = {
    {rt_r00, rt_r01, rt_px},
    {rt_r10, rt_r11, rt_py},
    {0.0, 0.0, 1.0}
  };

  T_camera_tag_ = {
    {ct_r00, ct_r01, ct_px},
    {ct_r10, ct_r11, ct_py},
    {0.0, 0.0, 1.0}
  };

  update_transform_();

  sub_pieces_pose_cam_ = create_subscription<tangram_msgs::msg::TangramPoses>(
    "pick/camera",
    10,
    std::bind(
      &FrameAlign::sub_pieces_pos_cam_callback_,
      this,
      std::placeholders::_1
    )
  );
  sub_apriltag_saved_ = create_subscription<tangram_msgs::msg::Point2D>(
    "april/saved",
    10,
    std::bind(
      &FrameAlign::sub_apriltag_saved_callback_,
      this,
      std::placeholders::_1
    )
  );

  pub_pieces_pose_robot_ = create_publisher<tangram_msgs::msg::TangramPoses>("pick/robot", 10);
}

void FrameAlign::sub_pieces_pos_cam_callback_(const tangram_msgs::msg::TangramPoses::SharedPtr msg)
{
  if (apriltag_ready_) {
    tangram_msgs::msg::TangramPoses msg_poses_robot;

    for (const auto & p : msg->poses) {
      const double x_cam = p.location.x;
      const double y_cam = p.location.y;

      const arma::colvec3 pvec_cam{x_cam, y_cam, 1.0};
      const arma::colvec3 pvev_robot = T_robot_camera_ * pvec_cam;

      const double x_robot = pvev_robot.at(0);
      const double y_robot = pvev_robot.at(1);

      tangram_msgs::msg::TangramPose pose;

      pose.location.x = x_robot;
      pose.location.y = y_robot;

      // pose.contour = p.contour;
      pose.type = p.type;
      pose.theta = p.theta;
      pose.uuid = p.uuid;

      msg_poses_robot.poses.push_back(pose);
    }

    pub_pieces_pose_robot_->publish(msg_poses_robot);
  }
}

void FrameAlign::sub_apriltag_saved_callback_(const tangram_msgs::msg::Point2D::SharedPtr msg)
{
  if (!apriltag_ready_) {
    const auto x = msg->x;
    const auto y = msg->y;

    T_camera_tag_.at(0, 2) = x;
    T_camera_tag_.at(1, 2) = y;

    update_transform_();

    apriltag_ready_ = true;
  }
}

arma::mat33 FrameAlign::trans_inv_(const arma::mat33 & T)
{
  const arma::mat22 R = T.submat(0, 0, 1, 1);
  const arma::colvec2 p = T.submat(0, 2, 1, 2);
  const arma::mat22 Rt = R.t();

  const arma::mat upper = arma::join_horiz(Rt, -Rt * p);
  const arma::rowvec3 lower{0.0, 0.0, 1.0};

  const arma::mat33 Tinv = arma::join_vert(upper, lower);
  return Tinv;
}

void FrameAlign::update_transform_()
{
  T_robot_camera_ = T_robot_camera_ = T_robot_tag_ * trans_inv_(T_camera_tag_);
  RCLCPP_INFO_STREAM(get_logger(), "Trc = " << std::endl << T_robot_camera_);
}
} /// namespace maxarm_control


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<maxarm_control::FrameAlign>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
