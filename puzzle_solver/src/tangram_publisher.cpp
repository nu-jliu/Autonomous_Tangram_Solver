#include <boost/filesystem.hpp>
#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "puzzle_solver/tangram_publisher.hpp"

namespace puzzle_solver
{
using namespace std::chrono_literals;

TangramPublisher::TangramPublisher()
: rclcpp::Node("tangram_image_publisher"), image_ready_(false)
{
  rcl_interfaces::msg::ParameterDescriptor image_dir_des;
  rcl_interfaces::msg::ParameterDescriptor image_name_des;

  image_dir_des.description = " Directory containing the source image";
  image_name_des.description = "File name of the image";

  declare_parameter<std::string>("image_dir", "", image_dir_des);
  declare_parameter<std::string>("image_name", "output_image.jpg", image_name_des);

  const std::string image_path = get_parameter("image_dir").as_string();
  const std::string image_name = get_parameter("image_name").as_string();

  const boost::filesystem::path directory(image_path);
  const boost::filesystem::path filename(image_name);
  const boost::filesystem::path full_path = directory / filename;
  image_full_path_ = full_path.string();

  timer_ = create_wall_timer(
    0.01s, std::bind(
      &TangramPublisher
      ::timer_callback_, this));
  pub_image_inferenced_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/inferenced", 10);
}

void TangramPublisher::timer_callback_()
{
  try {
    image_ = cv::imread(image_full_path_, cv::IMREAD_GRAYSCALE);

    if (!image_ready_) {
      image_ready_ = true;
    }
  } catch (cv::Exception & e) {
    RCLCPP_WARN_STREAM(get_logger(), "Got exception: " << e.what());
  }


  const auto image_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    sensor_msgs::image_encodings::MONO8,
    image_
  ).toImageMsg();
  pub_image_inferenced_->publish(*image_msg);
}
} /// namespace puzzle_solver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<puzzle_solver::TangramPublisher
    >();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
