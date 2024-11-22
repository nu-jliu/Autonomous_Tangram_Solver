#include <memory>
#include <chrono>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>

#include "tangram_detection/tangram_detection.hpp"
#include "tangram_detection/tangram_match.hpp"

namespace tangram_detection
{
using namespace std::chrono_literals;

TangramDetection::TangramDetection()
: rclcpp::Node("tangram_detection"), image_ready_(false)
{
  timer_ = create_wall_timer(0.5s, std::bind(&TangramDetection::timer_callback_, this));

  sub_image_segment_ =
    create_subscription<sensor_msgs::msg::Image>(
    "tangram/image/segment", 10,
    std::bind(
      &TangramDetection::sub_tangram_image_segment_callback_,
      this,
      std::placeholders::_1
    ));

  pub_image_source_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/source", 10);
  pub_image_dialate_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/dilate", 10);
  pub_image_erode_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/erode", 10);
  pub_image_opened_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/opened", 10);
  pub_image_closed_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/closed", 10);
  pub_image_edges_ = create_publisher<sensor_msgs::msg::Image>("tangram/image/edges", 10);
  pub_image_contours_raw_ = create_publisher<sensor_msgs::msg::Image>(
    "tangram/image/contours/raw",
    10
  );
  pub_image_contours_approx_ = create_publisher<sensor_msgs::msg::Image>(
    "tangram/image/contours/approx",
    10
  );
}

TangramDetection::~TangramDetection()
{
  RCLCPP_INFO(get_logger(), "Shutting down the node");
}

void TangramDetection::timer_callback_()
{
  if (image_ready_) {

    try {

      cv::Mat erode, dilate, opened, closed, edges, contours_raw, contours_approx;
      int kernelSize = 3; // Size of the structuring element
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

      cv::erode(source_img_, erode, kernel, cv::Point(-1, -1), 2);
      cv::dilate(source_img_, dilate, kernel);
      cv::morphologyEx(erode, opened, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, kernel);

      // Perform Canny edge detection
      const double lowThreshold = 50.0;
      const double highThreshold = 150.0;
      // int kernelSize = 3;   // Kernel size for the Sobel operator
      cv::Canny(closed, edges, lowThreshold, highThreshold);

      // Extract contours from the edge-detected image
      std::vector<std::vector<cv::Point>> contours;
      // std::vector<cv::Vec4i> hierarchy; // This can be ignored if hierarchy isn't needed
      cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // // Print the contours as lists of (x, y) coordinates
      // for (size_t i = 0; i < contours.size(); ++i) {
      //   RCLCPP_INFO_STREAM(get_logger(), "Contour " << i + 1 << ": ");
      //   for (const auto & point : contours[i]) {
      //     RCLCPP_INFO_STREAM(get_logger(), "(" << point.x << ", " << point.y << ") ");
      //   }
      //   // std::cout << std::endl;
      // }

      // RCLCPP_INFO_STREAM(get_logger(), edges.size());
      contours_raw = cv::Mat::zeros(edges.size(), CV_8UC3);
      cv::RNG rng(12345); // Random color generator for contours
      for (size_t i = 0; i < contours.size(); ++i) {
        const cv::Scalar color =
          cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::drawContours(contours_raw, contours, (int)i, color, 1);
        // cv::polylines(target_img_, contours.at(i), true, color);
      }

      contours_approx = cv::Mat::zeros(edges.size(), CV_8UC3);
      for (const auto & contour : contours) {
        // Approximate the contour
        std::vector<cv::Point> approxContour;
        const double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approxContour, epsilon, true);

        const std::string name = closest_tangram_piece_name(contour);

        const cv::Rect bounding_box = cv::boundingRect(contour);
        const int w = bounding_box.width;
        const int h = bounding_box.height;

        std::stringstream ss_size("");
        ss_size << w << ", " << h;
        const std::string text_size = ss_size.str();

        const cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::polylines(contours_approx, contour, true, color);

        const cv::Moments m = cv::moments(contour);

        if (m.m00 != 0) {
          const int cx = static_cast<int>(m.m10 / m.m00);
          const int cy = static_cast<int>(m.m01 / m.m00);

          const cv::Point center(cx, cy);
          const cv::Point center_up(cx - 20, cy);
          const cv::Point center_down(cx - 20, cy + 10);
          const cv::Point center_lower(cx - 20, cy + 20);

          std::stringstream ss_center("");
          ss_center << cx << ", " << cy;
          const std::string text_center = ss_center.str();

          const cv::Scalar color_text(255, 255, 255);

          cv::circle(contours_approx, center, 5, color, -1);

          const double scale = 0.3;
          cv::putText(
            contours_approx,
            text_size,
            center_up,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
          cv::putText(
            contours_approx,
            text_center,
            center_down,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
          cv::putText(
            contours_approx,
            name,
            center_lower,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
        }

        // // Find the closest Tangram piece
        // int closestPiece = findClosestTangramPiece(approxContour, tangramPieces);
        // matchedTangramIndices.push_back(closestPiece);
      }

      const auto image_source = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        source_img_
      ).toImageMsg();
      pub_image_source_->publish(*image_source);

      const auto image_erode = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        erode
      ).toImageMsg();
      pub_image_erode_->publish(*image_erode);

      const auto image_dilate = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        dilate
      ).toImageMsg();
      pub_image_dialate_->publish(*image_dilate);

      const auto image_opened = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        opened
      ).toImageMsg();
      pub_image_opened_->publish(*image_opened);

      const auto image_closed = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        closed
      ).toImageMsg();
      pub_image_closed_->publish(*image_closed);

      const auto image_edges = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        edges
      ).toImageMsg();
      pub_image_edges_->publish(*image_edges);

      const auto image_contour_raw = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        contours_raw
      ).toImageMsg();
      pub_image_contours_raw_->publish(*image_contour_raw);

      const auto image_contour_approx = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        contours_approx
      ).toImageMsg();
      pub_image_contours_approx_->publish(*image_contour_approx);

    } catch (cv::Exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Got Exception: " << e.what());
    }
  }
}

void TangramDetection::sub_tangram_image_segment_callback_(sensor_msgs::msg::Image::SharedPtr msg)
{
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::MONO8
  );
  source_img_ = image_ptr->image;

  if (!image_ready_) {
    image_ready_ = true;
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
