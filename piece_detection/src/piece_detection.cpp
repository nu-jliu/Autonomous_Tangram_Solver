/// \file piece_detection.cpp
/// \author Allen Liu (jingkunliu2025@u.northwestern.edu)
/// \brief Detect the tangram piece
///
/// SERVICES:
///   \li piece/detection/reset: Reset the node
///
/// SUBSCRIPTIONS:
///   \li image/piece/segment: Segmeneted tangram piece image
///   \li image/piece/mask: Masked tangram piece image
///
/// PUBLISHERS:
///   \li image/piece/erodes: Eroded image
///   \li image/piece/erodes: Opened image
///   \li image/piece/erodes: Closed image
///   \li image/piece/erodes: Image after edge detection
///   \li image/piece/erodes: Raw contours
///   \li image/piece/erodes: Labeled contours
///   \li pick/pixel: Pick position in pixel
///
/// \version 0.1.1
/// \date 2024-12-11
///
/// \copyright Copyright (c) 2024
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "piece_detection/piece_detection.hpp"
#include "tangram_utils/tangram_match.hpp"
#include "tangram_msgs/msg/tangram_piece.hpp"
#include "tangram_msgs/msg/point2_d_int.hpp"

namespace piece_detection
{
using namespace std::chrono_literals;

PieceDetection::PieceDetection()
: rclcpp::Node("piece_detection"),
  segment_ready_(false), mask_ready_(false),
  rng_(0xffffffff), target_size_(640, 640)
{
  timer_ = create_wall_timer(0.01s, std::bind(&PieceDetection::timer_callback_, this));

  srv_reset_ = create_service<std_srvs::srv::Trigger>(
    "piece/detection/reset",
    std::bind(
      &PieceDetection::srv_reset_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  sub_image_segment_ = create_subscription<sensor_msgs::msg::Image>(
    "image/piece/segment",
    10,
    std::bind(
      &PieceDetection::sub_image_segment_callback_,
      this,
      std::placeholders::_1
  ));
  sub_image_mask_ = create_subscription<sensor_msgs::msg::Image>(
    "image/piece/mask",
    10,
    std::bind(
      &PieceDetection::sub_image_mask_callback_,
      this,
      std::placeholders::_1
  ));

  pub_image_erodes_ = create_publisher<sensor_msgs::msg::Image>("image/piece/erodes", 10);
  pub_image_opened_ = create_publisher<sensor_msgs::msg::Image>("image/piece/opened", 10);
  pub_image_closed_ = create_publisher<sensor_msgs::msg::Image>("image/piece/closed", 10);
  pub_image_edges_ = create_publisher<sensor_msgs::msg::Image>("image/piece/edges", 10);
  pub_image_contour_raw_ = create_publisher<sensor_msgs::msg::Image>("image/piece/contour/raw", 10);
  pub_image_contour_labeled_ = create_publisher<sensor_msgs::msg::Image>(
    "image/piece/contour/labeled",
    10
  );

  pub_piece_pixels_ = create_publisher<tangram_msgs::msg::TangramPieces>(
    "pick/pixel",
    10
  );
}

void PieceDetection::timer_callback_()
{
  if (segment_ready_ && mask_ready_) {

    try {
      const int row_size = image_segment_.rows;

      cv::Mat img_erode, img_opened, img_closed, img_edges, img_contours, img_labeled;
      tangram_msgs::msg::TangramPieces pieces_msg;

      const int kernelSize = 3; // Size of the structuring element
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernelSize, kernelSize)
      );

      cv::erode(image_mask_, img_erode, kernel, cv::Point(-1, -1), 3);
      cv::morphologyEx(img_erode, img_opened, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(img_opened, img_closed, cv::MORPH_CLOSE, kernel);

      // Perform Canny edge detection
      const double threshold_low = 50.0;
      const double threshold_high = 150.0;
      // int kernelSize = 3;   // Kernel size for the Sobel operator
      cv::Canny(img_closed, img_edges, threshold_low, threshold_high);

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

        cv::drawContours(img_contours, contours, static_cast<int>(i), color, 1);
      }

      std::vector<size_t> shapes;
      shapes.clear();

      for (const auto & contour : contours) {

        std::vector<cv::Point> approx_contour;
        const double eplison = 0.05 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx_contour, eplison, true);

        const size_t type = tangram_utils::find_closest_tangram_piece(approx_contour);
        const std::string name = tangram_utils::closest_tangram_piece_name(approx_contour);
        const double radian = tangram_utils::get_tangram_piece_orientation(
          approx_contour,
          img_labeled
        );
        const double degree = radian * 180.0 / CV_PI;
        const auto area = cv::contourArea(contour);

        shapes.push_back(type);

        std::stringstream ss_area("");
        ss_area << area;
        const std::string text_area = ss_area.str();

        std::stringstream ss_angle("");
        ss_angle << degree;
        const std::string text_angle = ss_angle.str();

        const cv::Scalar color(
          rng_.uniform(low, high),
          rng_.uniform(low, high),
          rng_.uniform(low, high)
        );
        cv::polylines(img_labeled, approx_contour, true, color);

        const cv::Point center = tangram_utils::find_center(approx_contour);

        const int cx = center.x;
        const int cy = center.y;
        const cv::Point center_up(cx - 50, cy);
        const cv::Point center_down(cx - 50, cy + 15);
        const cv::Point center_lower(cx - 50, cy + 30);

        tangram_msgs::msg::TangramPiece piece;

        piece.location.x = cx;
        piece.location.y = row_size - cy;

        piece.theta = radian;
        piece.type = static_cast<int32_t>(type);

        if (type < 3) {
          const bool flipped = tangram_utils::is_triangle_flipped(approx_contour);
          piece.flipped = flipped;
        } else {
          piece.flipped = false;
        }

        const boost::uuids::uuid uuid = uuid_genorator_();
        piece.uuid = boost::uuids::to_string(uuid);

        pieces_msg.pieces.push_back(piece);

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
        cv::putText(
          img_labeled,
          text_center,
          center_lower,
          cv::FONT_HERSHEY_SIMPLEX,
          scale,
          color_text,
          1
        );
        cv::putText(
          img_labeled,
          text_area,
          center_up,
          cv::FONT_HERSHEY_SIMPLEX,
          scale,
          color_text,
          1
        );
      }
      // }


      const auto image_erodes = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        img_erode
      ).toImageMsg();
      pub_image_erodes_->publish(*image_erodes);

      const auto image_opened = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        img_opened
      ).toImageMsg();
      pub_image_opened_->publish(*image_opened);

      const auto image_closed = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        img_closed
      ).toImageMsg();
      pub_image_closed_->publish(*image_closed);

      const auto image_edges = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::MONO8,
        img_edges
      ).toImageMsg();
      pub_image_edges_->publish(*image_edges);

      const auto image_contour_raw = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        img_contours
      ).toImageMsg();
      pub_image_contour_raw_->publish(*image_contour_raw);

      const auto image_contour_labeled = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        img_labeled
      ).toImageMsg();
      pub_image_contour_labeled_->publish(*image_contour_labeled);

      if (!tangram_utils::validate_pieces(shapes)) {
        RCLCPP_WARN_ONCE(get_logger(), "Invalid shapes");
        return;
      }

      pub_piece_pixels_->publish(pieces_msg);
    } catch (std::exception & e) {
      RCLCPP_ERROR_STREAM_ONCE(
        get_logger(),
        "Got exception on line " << __LINE__ << ": " << e.what()
      );
    }
  } else {
    RCLCPP_WARN_STREAM_ONCE(get_logger(), "Segmented or mask image not ready yet");
  }
}

void PieceDetection::srv_reset_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;

  segment_ready_ = false;
  mask_ready_ = false;

  RCLCPP_INFO(get_logger(), "Node resetted");
  response->success = true;
}

void PieceDetection::sub_image_segment_callback_(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received segmented image");

  cv::Mat image_segment_raw;
  cv_bridge::CvImage::Ptr cv_image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::BGR8
  );
  image_segment_ = cv_image_ptr->image;

  if (!segment_ready_) {
    segment_ready_ = true;
  }
}

void PieceDetection::sub_image_mask_callback_(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received mask image");

  cv::Mat image_mask_raw, image_mask_resized;
  cv_bridge::CvImage::Ptr cv_image_ptr = cv_bridge::toCvCopy(
    msg,
    sensor_msgs::image_encodings::MONO8
  );

  image_mask_ = cv_image_ptr->image;

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
  rclcpp::shutdown();
  return 0;
}
