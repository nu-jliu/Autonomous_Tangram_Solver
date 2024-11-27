#include <chrono>
#include <memory>

#include "piece_detection/rs_pixel_to_real.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "tangram_msgs/msg/tangram_pose.hpp"

namespace piece_detection
{
using namespace std::chrono_literals;

RealSensePixelToReal::RealSensePixelToReal()
: rclcpp::Node("rs_pixel_2_real"), depth_scale_(0.0010000000474974513f),
  depth_img_ready_(false), camera_info_ready_(false), pieces_pixel_ready_(false)
{
  depth_intrinsics_ = std::make_shared<rs2_intrinsics>();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = create_wall_timer(0.01s, std::bind(&RealSensePixelToReal::timer_callback_, this));

  sub_image_aligned_depth_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/aligned_depth_to_color/image_raw",
    10,
    std::bind(
      &RealSensePixelToReal::sub_image_aligned_depth_callback_,
      this,
      std::placeholders::_1
    )
  );

  sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/aligned_depth_to_color/camera_info",
    10,
    std::bind(
      &RealSensePixelToReal::sub_camera_info_callback_,
      this,
      std::placeholders::_1
    )
  );

  sub_tangram_pieces_pixel_ = create_subscription<tangram_msgs::msg::TangramPieces>(
    "pick/pixel",
    10,
    std::bind(
      &RealSensePixelToReal::sub_tangram_pieces_pixel_callback_,
      this,
      std::placeholders::_1
    )
  );

  pub_tangram_pieces_real_ = create_publisher<tangram_msgs::msg::TangramPoses>(
    "pick/camera",
    10
  );
}

void RealSensePixelToReal::timer_callback_()
{
  if (!depth_img_ready_) {
    RCLCPP_DEBUG(get_logger(), "Depth Image not ready yet");
    return;
  }

  if (!camera_info_ready_) {
    RCLCPP_DEBUG(get_logger(), "Camera info not ready yet");
    return;
  }

  if (!pieces_pixel_ready_) {
    RCLCPP_DEBUG(get_logger(), "Tangram pieces pixel location not ready yet");
    return;
  }

  tangram_msgs::msg::TangramPoses msg_poses;

  int index = 0;
  for (const auto & piece : tangram_pieces_pixel_->pieces) {
    const auto x = static_cast<float>(piece.location.x);
    const auto y = static_cast<float>(piece.location.y);
    const auto z = static_cast<float>(aligned_depth_img_->image.at<uint16_t>(x, y) * depth_scale_);

    std::vector<float> pixel{x, y};
    std::vector<float> point(3);

    rs2_deproject_pixel_to_point(point.data(), depth_intrinsics_.get(), pixel.data(), z);


    const auto real_x = point.at(0);
    const auto real_y = point.at(1);
    const auto real_z = point.at(2);

    tangram_msgs::msg::TangramPose pose;
    // pose.contour = piece.contour;
    pose.type = piece.type;
    pose.theta = piece.theta;
    pose.uuid = piece.uuid;

    pose.location.x = real_x;
    pose.location.y = real_y;

    msg_poses.poses.push_back(pose);

    std::stringstream ss_frame_id("");
    ss_frame_id << "tangram_" << ++index;

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = ss_frame_id.str();

    tf.transform.translation.x = real_x;
    tf.transform.translation.y = real_y;
    tf.transform.translation.z = real_z;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf);
  }

  pub_tangram_pieces_real_->publish(msg_poses);
}

void RealSensePixelToReal::sub_image_aligned_depth_callback_(sensor_msgs::msg::Image::SharedPtr msg)
{
  aligned_depth_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

  if (!depth_img_ready_) {
    depth_img_ready_ = true;
  }
}

void RealSensePixelToReal::sub_camera_info_callback_(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  frame_id_ = msg->header.frame_id;

  depth_intrinsics_->ppx = msg->k.at(2);
  depth_intrinsics_->ppy = msg->k.at(5);
  depth_intrinsics_->fx = msg->k.at(0);
  depth_intrinsics_->fy = msg->k.at(4);
  depth_intrinsics_->model = RS2_DISTORTION_BROWN_CONRADY;       // Assuming standard distortion model

  for (int i = 0; i < 5; ++i) {
    depth_intrinsics_->coeffs[i] = msg->d.at(i);
  }

  if (!camera_info_ready_) {
    camera_info_ready_ = true;
  }
}

void RealSensePixelToReal::sub_tangram_pieces_pixel_callback_(
  tangram_msgs::msg::TangramPieces::SharedPtr msg
)
{
  tangram_pieces_pixel_ = msg;

  if (!pieces_pixel_ready_) {
    pieces_pixel_ready_ = true;
  }
}
}  // namespace piece_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<piece_detection::RealSensePixelToReal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
