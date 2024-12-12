#ifndef PUZZLE_SOLVER__PUZZLE_SOLVER_HPP___
#define PUZZLE_SOLVER__PUZZLE_SOLVER_HPP___

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include "tangram_msgs/msg/tangram_pieces.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace puzzle_solver
{
class PuzzleSolver : public rclcpp::Node
{
public:
  /// \brief Constructor of the puzzle_solver node
  PuzzleSolver();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_inferenced_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_source_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_erode_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_dialate_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_opened_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_closed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_edges_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contours_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_contours_labeled_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPieces>::SharedPtr pub_tangram_pieces_;

  boost::uuids::random_generator_mt19937 uuid_generator_;

  bool image_ready_;
  cv::RNG rng_;
  cv::Mat source_img_;
  cv::Mat target_img_;

  /// \brief Timer callback function
  void timer_callback_();

  /// \brief Reset the node
  /// \param request Request object of the service
  /// \param response Response object of the service
  void srv_reset_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );

  /// \brief Subsciption callback of the interences tangram image
  /// \param msg The subscribed inferenced image object
  void sub_tangram_image_inferenced_callback_(sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace puzzle_solver
#endif  // PUZZLE_SOLVER__PUZZLE_SOLVER_HPP___
