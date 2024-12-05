#ifndef PIECE_DETECTION__RS_PIXEL_TO_REAL_HPP___
#define PIECE_DETECTION__RS_PIXEL_TO_REAL_HPP___

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "tangram_msgs/msg/tangram_pieces.hpp"
#include "tangram_msgs/msg/tangram_poses.hpp"

#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace piece_detection
{
class RealSensePixelToReal : public rclcpp::Node
{
public:
  RealSensePixelToReal();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_aligned_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Subscription<tangram_msgs::msg::TangramPieces>::SharedPtr sub_tangram_pieces_pixel_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_tangram_pieces_real_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // sensor_msgs::msg::Image::SharedPtr aligned_depth_img_;
  cv_bridge::CvImage::Ptr aligned_depth_img_;
  tangram_msgs::msg::TangramPieces::SharedPtr tangram_pieces_pixel_;
  std::shared_ptr<rs2_intrinsics> depth_intrinsics_;


  float depth_scale_;
  bool depth_img_ready_;
  bool camera_info_ready_;
  bool pieces_pixel_ready_;

  std::string frame_id_;


  void timer_callback_();

  void srv_reset_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );

  void sub_image_aligned_depth_callback_(const sensor_msgs::msg::Image::SharedPtr msg);

  void sub_camera_info_callback_(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void sub_tangram_pieces_pixel_callback_(const tangram_msgs::msg::TangramPieces::SharedPtr msg);
};
}  // namespace piece_detection
#endif  // PIECE_DETECTION__RS_PIXEL_TO_REAL_HPP___
