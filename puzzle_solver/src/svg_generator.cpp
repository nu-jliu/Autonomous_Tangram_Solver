#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "puzzle_solver/svg_generator.hpp"

namespace puzzle_solver
{
using namespace std::chrono_literals;

SVGGenerator::SVGGenerator()
: rclcpp::Node("svg_generator"), pieces_ready_(false)
{
  rcl_interfaces::msg::ParameterDescriptor svg_file_dir_des;
  rcl_interfaces::msg::ParameterDescriptor svg_filename_des;

  svg_file_dir_des.description = "Directory of the target svg file";
  svg_filename_des.description = "File name of the target svg file";

  declare_parameter<std::string>("svg_dir", "", svg_file_dir_des);
  declare_parameter<std::string>("svg_name", "", svg_filename_des);

  const std::string svg_file_dir = get_parameter("svg_dir").as_string();
  const std::string svg_filename = get_parameter("svg_name").as_string();

  const boost::filesystem::path directory(svg_file_dir);
  const boost::filesystem::path filename(svg_filename);
  const boost::filesystem::path full_path = directory / filename;

  svg_file_path_ = full_path.string();
}

void SVGGenerator::sub_puzzle_pixel_callback_(const tangram_msgs::msg::TangramPieces::SharedPtr msg)
{
  if (!pieces_ready_) {
    puzzle_pieces_ = msg->pieces;
    pieces_ready_ = true;
  }
}

void SVGGenerator::srv_generate_svg_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;
  RCLCPP_INFO(get_logger(), "Generating svg ...");

  std::this_thread::sleep_for(1s);

  try {

    std::ofstream svg_file(svg_file_path_);
    if (!svg_file.is_open()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open file " << svg_file_path_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Got exception: " << e.what());
    response->success = false;
  }

  RCLCPP_INFO(get_logger(), "Finished generating svg");
  response->success = true;
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<puzzle_solver::SVGGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
