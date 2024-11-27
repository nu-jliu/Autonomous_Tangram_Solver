#ifndef PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP___
#define PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP___

#include <rclcpp/rclcpp.hpp>

#include "tangram_msgs/msg/tangram_pieces.hpp"
#include "tangram_msgs/msg/tangram_poses.hpp"

namespace puzzle_solver
{
class SolutionPixelToReal : public rclcpp::Node
{
public:
  SolutionPixelToReal();

private:
  rclcpp::Subscription<tangram_msgs::msg::TangramPieces>::SharedPtr sub_tangram_pieces_pixel_;

  rclcpp::Publisher<tangram_msgs::msg::TangramPoses>::SharedPtr pub_tangram_pieces_real_;

  double scale_;
  double offset_;

  void sub_tangram_pieces_pixel_callback_(tangram_msgs::msg::TangramPieces::SharedPtr msg);
};
}  // namespace puzzle_solver
#endif  // PUZZLE_SOLVER__SOLUTION_PIXEL_TO_REAL_HPP_
