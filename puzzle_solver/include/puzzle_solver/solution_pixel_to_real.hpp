#ifndef PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP___
#define PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP___

#include <rclcpp/rclcpp.hpp>

#include "tangram_msgs/msg/tangram_pieces.hpp"
#include "tangram_msgs/msg/tangram_poses.hpp"

#include <std_srvs/srv/trigger.hpp>

namespace puzzle_solver
{
class SolutionPixelToReal : public rclcpp::Node
{
public:
  /// \brief The constructor of the solution_pixel_to_real node
  SolutionPixelToReal();

private:
  rclcpp::Subscription<tangram_msgs::msg::TangramPieces>::SharedPtr sub_tangram_pieces_pixel_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_tangram_pieces_real_;

  double scale_;
  double offset_;

  /// \brief Subcription callback function of the tangram piece pixel position
  /// \param msg The subcribed tangram pieces position object
  void sub_tangram_pieces_pixel_callback_(tangram_msgs::msg::TangramPieces::SharedPtr msg);
};
}  // namespace puzzle_solver
#endif  // PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP_
