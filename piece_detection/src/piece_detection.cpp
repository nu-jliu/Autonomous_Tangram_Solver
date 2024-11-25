#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "piece_detection/piece_detection.hpp"
#include "tangram_utils/tangram_match.hpp"

namespace piece_detection
{
using namespace std::chrono_literals;

PieceDetection::PieceDetection()
: rclcpp::Node("piece_detection"),
  segment_ready_(false), mask_ready_(false),
  rng_(0xffffffff), target_size_(640, 640)
{
  timer_ = create_wall_timer(0.01s, std::bind(&PieceDetection::timer_callback_, this));

  sub_image_piece_segment_ = create_subscription<sensor_msgs::msg::Image>(
    "image/piece/segment",
    10,
    std::bind(
      &PieceDetection::sub_image_piece_segment_callback_,
      this,
      std::placeholders::_1
  ));
  sub_image_piece_mask_ = create_subscription<sensor_msgs::msg::Image>(
    "image/piece/mask",
    10,
    std::bind(
      &PieceDetection::sub_image_piece_mask_callback_,
      this,
      std::placeholders::_1
  ));

  pub_image_piece_edges_ = create_publisher<sensor_msgs::msg::Image>(
    "image/piece/edges",
    10
  );
  pub_image_piece_contour_raw_ = create_publisher<sensor_msgs::msg::Image>(
    "image/piece/contour/raw",
    10
  );
  pub_image_piece_contour_labeled_ = create_publisher<sensor_msgs::msg::Image>(
    "image/piece/contour/labeled",
    10
  );
}

void PieceDetection::timer_callback_()
{
  if (segment_ready_ && mask_ready_) {
    // Do something
    cv::Mat img_edges, img_contours, img_labeled;

    // Perform Canny edge detection
    const double threshold_low = 50.0;
    const double threshold_high = 150.0;
    // int kernelSize = 3;   // Kernel size for the Sobel operator
    cv::Canny(image_mask_, img_edges, threshold_low, threshold_high);

    img_contours = cv::Mat::zeros(image_mask_.size(), CV_8UC3);
    img_labeled = cv::Mat::zeros(image_mask_.size(), CV_8UC3);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const int low = 127;
    const int high = 256;

    for (size_t i = 0; i < contours.size(); ++i) {
      const cv::Scalar color(
        rng_.uniform(low, high),
        rng_.uniform(low, high),
        rng_.uniform(low, high)
      );

      cv::drawContours(img_contours, contours, static_cast<int>(i), color, i);
    }

    for (const auto & contour : contours) {
      std::vector<cv::Point> approx_contour;
      const double eplison = 0.02 * cv::arcLength(contour, true);
      cv::approxPolyDP(contour, approx_contour, eplison, true);

      const cv::Scalar color(
        rng_.uniform(low, high),
        rng_.uniform(low, high),
        rng_.uniform(low, high)
      );
      cv::polylines(img_labeled, approx_contour, true, color);

      const cv::Moments m = cv::moments(approx_contour);

      if (m.m00 != 0) {
        const int cx = static_cast<int>(m.m10 / m.m00);
        const int cy = static_cast<int>(m.m01 / m.m00);

        const cv::Point center(cx, cy);
        const cv::Point center_up(cx - 50, cy);
        const cv::Point center_down(cx - 50, cy + 15);
        const cv::Point center_lower(cx - 50, cy + 30);

        const std::string name = tangram_utils::closest_tangram_piece_name(approx_contour);

        std::stringstream ss_center("");
        ss_center << cx << ", " << cy;
        const std::string text_center = ss_center.str();

        const cv::Scalar color_text(255, 255, 255);

        const double scale = 0.5;
        cv::circle(img_labeled, center, 5, color, -1);
        cv::putText(
          img_labeled,
          name,
          center_down,
          cv::FONT_HERSHEY_SIMPLEX,
          scale,
          color_text,
          1
        );
      }
    }

    const auto image_edges = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      sensor_msgs::image_encodings::MONO8,
      img_edges
    ).toImageMsg();
    pub_image_piece_edges_->publish(*image_edges);

    const auto image_contour_raw = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      sensor_msgs::image_encodings::BGR8,
      img_contours
    ).toImageMsg();
    pub_image_piece_contour_raw_->publish(*image_contour_raw);

    const auto image_contour_labeled = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      sensor_msgs::image_encodings::BGR8,
      img_labeled
    ).toImageMsg();
    pub_image_piece_contour_labeled_->publish(*image_contour_labeled);
  } else {
    RCLCPP_WARN_STREAM_ONCE(get_logger(), "Segmented or mask image not ready yet");
  }
}

void PieceDetection::sub_image_piece_segment_callback_(sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received segmented image");

  cv::Mat image_segment_raw;
  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::BGR8
  );
  image_segment_raw = cv_image_ptr->image;
  cv::resize(image_segment_raw, image_segment_, target_size_, 0.0, 0.0, cv::INTER_AREA);

  if (!segment_ready_) {
    segment_ready_ = true;
  }
}

void PieceDetection::sub_image_piece_mask_callback_(sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat image_mask_raw, image_mask_resized;
  RCLCPP_INFO(get_logger(), "Received mask image");
  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::MONO8
  );
  image_mask_raw = cv_image_ptr->image;
  cv::resize(image_mask_raw, image_mask_resized, target_size_, 0.0, 0.0, cv::INTER_NEAREST);
  cv::threshold(image_mask_resized, image_mask_, 127, 255, cv::THRESH_BINARY);

  if (!mask_ready_) {
    mask_ready_ = true;
  }
}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<piece_detection::PieceDetection>();
  rclcpp::spin(node);
  // auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // executor->add_node(node);
  // executor->spin();
  rclcpp::shutdown();
  return 0;
}
