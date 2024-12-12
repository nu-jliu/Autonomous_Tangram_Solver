#ifndef PIECE_DETECTION__PIECE_DETECTION_HPP___
#define PIECE_DETECTION__PIECE_DETECTION_HPP___

#include <opencv2/opencv.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include "tangram_msgs/msg/tangram_pieces.hpp"

#include <std_srvs/srv/trigger.hpp>

namespace piece_detection
{
class PieceDetection : public rclcpp::Node
{
public:
  /// \brief Construct the piece_detection node
  PieceDetection();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_segment_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_mask_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_erodes_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_opened_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_closed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_edges_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contour_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contour_labeled_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPieces>::SharedPtr pub_piece_pixels_;

  boost::uuids::random_generator_mt19937 uuid_genorator_;

  bool segment_ready_;
  bool mask_ready_;

  cv::RNG rng_;
  cv::Size target_size_;
  cv::Mat image_segment_;
  cv::Mat image_mask_;

  /// \brief Timer callback function
  void timer_callback_();

  /// \brief Reset service callback
  /// \param request Request object of the service
  /// \param response Response object of the service
  void srv_reset_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );

  /// \brief Subcription callback of the segmented image
  /// \param msg Subcribed image message object
  void sub_image_segment_callback_(const sensor_msgs::msg::Image::SharedPtr msg);

  /// \brief Subcription callback of the masked image
  /// \param msg Subcribed image message object
  void sub_image_mask_callback_(const sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace piece_detection
#endif  // PIECE_DETECTION__PIECE_DETECTION_HPP___
