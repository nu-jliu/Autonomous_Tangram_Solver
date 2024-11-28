#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>

#include "puzzle_solver/puzzle_solver.hpp"
// #include "puzzle_solver/tangram_match.hpp"
#include "tangram_utils/tangram_match.hpp"
#include "tangram_msgs/msg/tangram_piece.hpp"

namespace puzzle_solver
{
using namespace std::chrono_literals;

TangramSolver::TangramSolver()
: rclcpp::Node("puzzle_solver"), image_ready_(false), rng_(0xffffffff)
{
  timer_ = create_wall_timer(0.01s, std::bind(&TangramSolver::timer_callback_, this));

  sub_image_inferenced_ =
    create_subscription<sensor_msgs::msg::Image>(
    "puzzle/image/inferenced", 10,
    std::bind(
      &TangramSolver::sub_tangram_image_inferenced_callback_,
      this,
      std::placeholders::_1
    ));

  pub_image_source_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/source", 10);
  pub_image_dialate_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/dilate", 10);
  pub_image_erode_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/erode", 10);
  pub_image_opened_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/opened", 10);
  pub_image_closed_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/closed", 10);
  pub_image_edges_ = create_publisher<sensor_msgs::msg::Image>("puzzle/image/edges", 10);
  pub_image_contours_raw_ = create_publisher<sensor_msgs::msg::Image>(
    "puzzle/image/contours/raw",
    10
  );
  pub_image_contours_labeled_ = create_publisher<sensor_msgs::msg::Image>(
    "puzzle/image/contours/labeled",
    10
  );

  pub_tangram_pieces_ = create_publisher<tangram_msgs::msg::TangramPieces>("place/pixel", 10);
}

TangramSolver::~TangramSolver()
{
  RCLCPP_INFO(get_logger(), "Shutting down the node");
}

void TangramSolver::timer_callback_()
{
  if (image_ready_) {

    try {
      const int row_size = source_img_.rows;
      const int col_size = source_img_.cols;

      tangram_msgs::msg::TangramPieces pieces_msg;

      cv::Mat erode, dilate, opened, closed, edges, contours_raw, contours_labeled;
      int kernelSize = 3; // Size of the structuring element
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

      cv::erode(source_img_, erode, kernel, cv::Point(-1, -1), 3);
      cv::dilate(source_img_, dilate, kernel);
      cv::morphologyEx(erode, opened, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, kernel);

      // Perform Canny edge detection
      const double threshold_low = 50.0;
      const double threshold_high = 150.0;
      // int kernelSize = 3;   // Kernel size for the Sobel operator
      cv::Canny(closed, edges, threshold_low, threshold_high);

      // Extract contours from the edge-detected image
      std::vector<std::vector<cv::Point>> contours;
      // std::vector<cv::Vec4i> hierarchy; // This can be ignored if hierarchy isn't needed
      cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      if (contours.size() != 7ul) {
        RCLCPP_DEBUG(get_logger(), "Invalid contour detected, skipped");
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

      const int low = 127;
      const int high = 256;

      // RCLCPP_INFO_STREAM(get_logger(), edges.size());
      contours_raw = cv::Mat::zeros(edges.size(), CV_8UC3);
      // cv::RNG rng(12345); // Random color generator for contours
      for (size_t i = 0; i < contours.size(); ++i) {


        const cv::Scalar color(
          rng_.uniform(low, high),
          rng_.uniform(low, high),
          rng_.uniform(low, high)
        );
        cv::drawContours(contours_raw, contours, static_cast<int>(i), color, 1);
        // cv::polylines(target_img_, contours.at(i), true, color);
      }

      std::vector<size_t> shapes;
      shapes.clear();

      contours_labeled = cv::Mat::zeros(edges.size(), CV_8UC3);
      for (const auto & contour : contours) {
        // Approximate the contour
        std::vector<cv::Point> approx_contour;
        const double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx_contour, epsilon, true);

        const double area = cv::contourArea(approx_contour);

        const size_t type = tangram_utils::find_closest_tangram_piece(approx_contour);
        const std::string name = tangram_utils::closest_tangram_piece_name(approx_contour);
        const double radian = tangram_utils::get_tangram_piece_orientation(
          approx_contour,
          contours_labeled
        );
        const double degree = radian * 180.0 / CV_PI;

        shapes.push_back(type);

        std::stringstream ss_area("");
        ss_area << area;
        const std::string text_area = ss_area.str();

        std::stringstream ss_degree("");
        ss_degree << degree;
        const std::string text_degree = ss_degree.str();

        const cv::Rect bounding_box = cv::boundingRect(approx_contour);
        const int w = bounding_box.width;
        const int h = bounding_box.height;

        std::stringstream ss_size("");
        ss_size << w << ", " << h;
        const std::string text_size = ss_size.str();

        const cv::Scalar color(
          rng_.uniform(low, high),
          rng_.uniform(low, high),
          rng_.uniform(low, high)
        );
        cv::polylines(contours_labeled, approx_contour, true, color);

        const cv::Moments m = cv::moments(approx_contour);

        if (m.m00 != 0) {
          const int cx = static_cast<int>(m.m10 / m.m00);
          const int cy = static_cast<int>(m.m01 / m.m00);

          const cv::Point center(cx, cy);
          const cv::Point center_up(cx - 50, cy);
          const cv::Point center_down(cx - 50, cy + 15);
          const cv::Point center_lower(cx - 50, cy + 30);

          tangram_msgs::msg::TangramPiece piece;

          piece.location.x = static_cast<int32_t>(cx - col_size / 2);
          piece.location.y = static_cast<int32_t>(cy - row_size / 2);

          piece.theta = radian;
          piece.type = static_cast<int32_t>(type);

          const boost::uuids::uuid uuid = uuid_generator_();
          piece.uuid = boost::uuids::to_string(uuid);

          pieces_msg.pieces.push_back(piece);

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
          cv::putText(
            contours_labeled,
            text_degree,
            center_lower,
            cv::FONT_HERSHEY_SIMPLEX,
            scale,
            color_text,
            1
          );
        }

        // // Find the closest Tangram piece
        // int closestPiece = findClosestTangramPiece(approx_contour, tangramPieces);
        // matchedTangramIndices.push_back(closestPiece);
      }

      if (!tangram_utils::validate_pieces(shapes)) {
        RCLCPP_WARN_ONCE(get_logger(), "Invalid shapes");
        return;
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

      pub_tangram_pieces_->publish(pieces_msg);
    } catch (std::exception & e) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "Got Exception: " << e.what());
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Image not ready yet");
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
