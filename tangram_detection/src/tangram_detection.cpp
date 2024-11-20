#include <memory>
#include <chrono>

#include "tangram_detection/tangram_detection.hpp"
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>

namespace tangram_detection
{
using namespace std::chrono_literals;

TangramDetection::TangramDetection()
: rclcpp::Node("tangram_detection")
{
  image_dir_ =
    "/home/jingkun/Documents/Final_Project/src/Autonomous_Tangram_Solver/model/test";
  boost::filesystem::path image_dir_path(image_dir_);
  boost::filesystem::path source_image_file("output_image.jpg");
  boost::filesystem::path target_image_file("target.png");
  image_dir_ = (image_dir_path / source_image_file).string();

  timer_ = create_wall_timer(0.1s, std::bind(&TangramDetection::timer_callback_, this));
  pub_image_ = create_publisher<sensor_msgs::msg::Image>("tangram/image", 10);
}

TangramDetection::~TangramDetection()
{
  RCLCPP_INFO(get_logger(), "Shutting down the node");
}

void TangramDetection::timer_callback_()
{
  source_img_ = cv::imread(image_dir_, cv::IMREAD_GRAYSCALE);

  try {
    cv::Mat closed_image, edges;
    int kernelSize = 3; // Size of the structuring element
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(source_img_, closed_image, cv::MORPH_CLOSE, element);

    // Perform Canny edge detection
    double lowThreshold = 50.0;
    double highThreshold = 150.0;
    // int kernelSize = 3;   // Kernel size for the Sobel operator
    cv::Canny(closed_image, edges, lowThreshold, highThreshold);

    // Extract contours from the edge-detected image
    std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy; // This can be ignored if hierarchy isn't needed
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Print the contours as lists of (x, y) coordinates
    for (size_t i = 0; i < contours.size(); ++i) {
      RCLCPP_INFO_STREAM(get_logger(), "Contour " << i + 1 << ": ");
      for (const auto & point : contours[i]) {
        RCLCPP_INFO_STREAM(get_logger(), "(" << point.x << ", " << point.y << ") ");
      }
      // std::cout << std::endl;
    }

    RCLCPP_INFO_STREAM(get_logger(), edges.size());
    target_img_ = cv::Mat::zeros(edges.size(), CV_8UC3);
    cv::RNG rng(12345); // Random color generator for contours
    for (size_t i = 0; i < contours.size(); ++i) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      cv::drawContours(target_img_, contours, (int)i, color, 1);
      // cv::polylines(target_img_, contours.at(i), true, color);
    }

    const auto image_tangram = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      sensor_msgs::image_encodings::BGR8,
      target_img_
    ).toImageMsg();
    pub_image_->publish(*image_tangram);
  } catch (std::exception & e) {
    // const auto image_tangram = cv_bridge::CvImage(
    //   std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, target_img_).toImageMsg();
    // pub_image_->publish(*image_tangram);
    RCLCPP_WARN_STREAM(get_logger(), "Invalid image: " << e.what());
  }
}
}  // namespace tangram_detection


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tangram_detection::TangramDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
