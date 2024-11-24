#include <memory>
#include <chrono>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>

#include "puzzle_solver/puzzle_solver.hpp"
#include "puzzle_solver/tangram_match.hpp"

namespace puzzle_solver
{
using namespace std::chrono_literals;

TangramSolver::TangramSolver()
: rclcpp::Node("puzzle_solver"), image_ready_(false)
{
  timer_ = create_wall_timer(0.1s, std::bind(&TangramSolver::timer_callback_, this));

  sub_image_inferenced_ =
    create_subscription<sensor_msgs::msg::Image>(
    "tangram/image/inferenced", 10,
    std::bind(
      &TangramSolver::sub_tangram_image_inferenced_callback_,
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
  pub_image_contours_labeled_ = create_publisher<sensor_msgs::msg::Image>(
    "tangram/image/contours/labeled",
    10
  );
}

TangramSolver::~TangramSolver()
{
  RCLCPP_INFO(get_logger(), "Shutting down the node");
}

void TangramSolver::timer_callback_()
{
  if (image_ready_) {

    try {

      cv::Mat erode, dilate, opened, closed, edges, contours_raw, contours_labeled;
      int kernelSize = 3; // Size of the structuring element
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

      cv::erode(source_img_, erode, kernel, cv::Point(-1, -1), 3);
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

      if (contours.size() < 7ul) {
        RCLCPP_WARN(get_logger(), "Not enough contour detected, skipped");
        return;
      }
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
        const int low = 127;
        const int high = 256;

        const cv::Scalar color = cv::Scalar(
          rng.uniform(low, high),
          rng.uniform(low, high),
          rng.uniform(low, high)
        );
        cv::drawContours(contours_raw, contours, (int)i, color, 1);
        // cv::polylines(target_img_, contours.at(i), true, color);
      }

      contours_labeled = cv::Mat::zeros(edges.size(), CV_8UC3);
      for (const auto & contour : contours) {
        // Approximate the contour
        std::vector<cv::Point> approxContour;
        const double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approxContour, epsilon, true);

        const std::string name = closest_tangram_piece_name(approxContour);

        const double area = cv::contourArea(approxContour);

        std::stringstream ss_area("");
        ss_area << area;
        const std::string text_area = ss_area.str();

        const cv::Rect bounding_box = cv::boundingRect(approxContour);
        const int w = bounding_box.width;
        const int h = bounding_box.height;

        std::stringstream ss_size("");
        ss_size << w << ", " << h;
        const std::string text_size = ss_size.str();

        const cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::polylines(contours_labeled, contour, true, color);

        const cv::Moments m = cv::moments(contour);

        if (m.m00 != 0) {
          const int cx = static_cast<int>(m.m10 / m.m00);
          const int cy = static_cast<int>(m.m01 / m.m00);

          const cv::Point center(cx, cy);
          const cv::Point center_up(cx - 50, cy);
          const cv::Point center_down(cx - 50, cy + 15);
          const cv::Point center_lower(cx - 50, cy + 30);

          std::stringstream ss_center("");
          ss_center << cx << ", " << cy;
          const std::string text_center = ss_center.str();

          const cv::Scalar color_text(255, 255, 255);

          cv::circle(contours_labeled, center, 5, color, -1);

          const double scale = 0.5;
          cv::putText(
            contours_labeled,
            text_center,
            center_up,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
          cv::putText(
            contours_labeled,
            name,
            center_down,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
          // cv::putText(
          //   contours_approx,
          //   name,
          //   center_lower,
          //   cv::FONT_HERSHEY_SIMPLEX,
          //   scale,
          //   color_text,
          //   1
          // );
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

      const auto image_contour_labeled = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        contours_labeled
      ).toImageMsg();
      pub_image_contours_labeled_->publish(*image_contour_labeled);

    } catch (cv::Exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Got Exception: " << e.what());
    }
  }
}

void TangramSolver::sub_tangram_image_inferenced_callback_(sensor_msgs::msg::Image::SharedPtr msg)
{
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::MONO8
  );
  const auto cv_img = image_ptr->image;
  cv::Mat resized;
  cv::resize(cv_img, resized, {640, 640}, 0.0, 0.0, cv::INTER_NEAREST);

  cv::threshold(resized, source_img_, 127, 255, cv::THRESH_BINARY);

  if (!image_ready_) {
    image_ready_ = true;
  }
}
}  /// namespace puzzle_solver


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<puzzle_solver::TangramSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
