#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

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
  tangram_pieces_pixel_ = std::make_shared<tangram_msgs::msg::TangramPieces>();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = create_wall_timer(0.01s, std::bind(&RealSensePixelToReal::timer_callback_, this));

  srv_reset_ = create_service<std_srvs::srv::Trigger>(
    "piece/p2r/reset",
    std::bind(
      &RealSensePixelToReal::srv_reset_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  sub_image_aligned_depth_ = create_subscription<sensor_msgs::msg::Image>(
    "/piece/camera/aligned_depth_to_color/image_raw",
    10,
    std::bind(
      &RealSensePixelToReal::sub_image_aligned_depth_callback_,
      this,
      std::placeholders::_1
    )
  );

  sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/piece/camera/aligned_depth_to_color/camera_info",
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

  try {
    int index = 0;
    tangram_msgs::msg::TangramPoses msg_poses;

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "got tangram pieces: " << static_cast<int>(tangram_pieces_pixel_->pieces.size())
    );

    for (const auto & piece : tangram_pieces_pixel_->pieces) {
      const auto x_pixel = static_cast<int>(piece.location.x);
      const auto y_pixel = static_cast<int>(piece.location.y);

      RCLCPP_DEBUG_STREAM(get_logger(), "Size: " << aligned_depth_img_->image.size);
      RCLCPP_DEBUG_STREAM(get_logger(), "Location: " << x_pixel << ", " << y_pixel);

      const auto x = static_cast<float>(x_pixel);
      const auto y = static_cast<float>(y_pixel);
      const auto z = static_cast<float>(
        aligned_depth_img_->image.at<uint16_t>(y_pixel, x_pixel) * depth_scale_
      );

      std::vector<float> pixel{x, y};
      std::vector<float> point(3);

      rs2_deproject_pixel_to_point(point.data(), depth_intrinsics_.get(), pixel.data(), z);

      const auto real_x = point.at(0);
      const auto real_y = point.at(1);
      const auto real_z = point.at(2);

      tangram_msgs::msg::TangramPose pose;

      pose.uuid = piece.uuid;
      pose.type = piece.type;
      pose.theta = piece.theta;
      pose.flipped = piece.flipped;

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
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM_ONCE(
      get_logger(),
      "Got Exception on line " << __LINE__ << ": " << e.what()
    );
  }
}

void RealSensePixelToReal::srv_reset_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;

  depth_img_ready_ = false;
  camera_info_ready_ = false;
  pieces_pixel_ready_ = false;

  RCLCPP_INFO(get_logger(), "Node resetted");
  response->success = true;
}

void RealSensePixelToReal::sub_image_aligned_depth_callback_(
  const sensor_msgs::msg::Image::SharedPtr msg
)
{
  aligned_depth_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

  if (!depth_img_ready_) {
    depth_img_ready_ = true;
  }
}

void RealSensePixelToReal::sub_camera_info_callback_(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg
)
{
  if (!camera_info_ready_) {
    frame_id_ = msg->header.frame_id;

    depth_intrinsics_->width = msg->width;
    depth_intrinsics_->height = msg->height;
    depth_intrinsics_->ppx = msg->k.at(2);
    depth_intrinsics_->ppy = msg->k.at(5);
    depth_intrinsics_->fx = msg->k.at(0);
    depth_intrinsics_->fy = msg->k.at(4);
    depth_intrinsics_->model = RS2_DISTORTION_BROWN_CONRADY;     // Assuming standard distortion model

    for (int i = 0; i < 5; ++i) {
      depth_intrinsics_->coeffs[i] = msg->d.at(i);
    }

    camera_info_ready_ = true;
  }
}

void RealSensePixelToReal::sub_tangram_pieces_pixel_callback_(
  const tangram_msgs::msg::TangramPieces::SharedPtr msg
)
{
  tangram_pieces_pixel_->pieces.clear();
  for (const auto & piece : msg->pieces) {
    tangram_pieces_pixel_->pieces.push_back(piece);
  }

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
