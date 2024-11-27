#ifndef PIECE_DETECTION__PIECE_DETECTION_HPP___
#define PIECE_DETECTION__PIECE_DETECTION_HPP___

#include <opencv2/opencv.hpp>
// #include "uuid_v4/uuid_v4.h"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include "tangram_msgs/msg/tangram_pieces.hpp"


namespace piece_detection
{
class PieceDetection : public rclcpp::Node
{
public:
  PieceDetection();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_piece_segment_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_piece_mask_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_piece_edges_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_piece_contour_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_piece_contour_labeled_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPieces>::SharedPtr pub_piece_pixels_;

  boost::uuids::random_generator_mt19937 uuid_genorator_;
  // UUIDv4::UUIDGenerator<std::mt19937_64> uuid_generator_;

  bool segment_ready_;
  bool mask_ready_;

  cv::RNG rng_;
  cv::Size target_size_;
  cv::Mat image_segment_;
  cv::Mat image_mask_;

  void timer_callback_();

  void sub_image_piece_segment_callback_(sensor_msgs::msg::Image::SharedPtr msg);

  void sub_image_piece_mask_callback_(sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace piece_detection
#endif  // PIECE_DETECTION__PIECE_DETECTION_HPP___
