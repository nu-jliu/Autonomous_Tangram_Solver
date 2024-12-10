#ifndef PUZZLE_SOLVER__SVG_GENERATOR_HPP___
#define PUZZLE_SOLVER__SVG_GENERATOR_HPP___

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>
#include "tangram_msgs/msg/tangram_pieces.hpp"

namespace puzzle_solver
{
class SVGGenerator : public rclcpp::Node
{
public:
  SVGGenerator();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_generate_svg_;

  rclcpp::Subscription<tangram_msgs::msg::TangramPieces>::SharedPtr sub_puzzle_pixel_;

  bool pieces_ready_;
  std::vector<tangram_msgs::msg::TangramPiece> puzzle_pieces_;
  std::string svg_file_path_;

  void sub_puzzle_pixel_callback_(const tangram_msgs::msg::TangramPieces::SharedPtr msg);

  void srv_generate_svg_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );
};
}  // namespace puzzle_solver
#endif  // PUZZLE_SOLVER__SVG_GENERATOR_HPP___
