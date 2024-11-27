#ifndef TANGRAM_UTILS__TANGRAM_MATCH_HPP___
#define TANGRAM_UTILS__TANGRAM_MATCH_HPP___

#include <opencv2/opencv.hpp>
#include <vector>

namespace tangram_utils
{
/// \brief Threshold for differentiating small and medium triangles
constexpr double small_medium_triangle_threshold = 4000.0;

/// \brief Threshold for differentiating medium and large triangles
constexpr double medium_large_triangle_threshold = 12000.0;

/// \brief The shape of all tangram pieces
const std::vector<std::vector<cv::Point>> tangrams_pieces{
  /// Small triangle
  {cv::Point(0, 0), cv::Point(23, 0), cv::Point(0, 23)},

  /// Medium triangle
  {cv::Point(0, 0), cv::Point(37, 0), cv::Point(0, 37)},

  /// Large triangle
  {cv::Point(0, 0), cv::Point(63, 0), cv::Point(0, 63)},

  /// Square
  {cv::Point(0, 0), cv::Point(35, 0), cv::Point(35, 35), cv::Point(0, 35)},

  /// Parallelogram
  {cv::Point(0, 0), cv::Point(37, 0), cv::Point(60, 23), cv::Point(23, 23)}
};

/// \brief Name of all tangram pieces
const std::vector<std::string> tangram_names{
  "Small Triangle",
  "Medium Triangle",
  "Large Triangle",
  "Square",
  "Parallelogram"
};

/// \brief Draw the principle axis on the image
/// \param img Image to be drawn on
/// \param p The starting point
/// \param q The end point
/// \param scale Scale of the axis to be displayed
void draw_axis(
  const cv::Mat & img,
  cv::Point p,
  cv::Point q,
  const cv::Scalar & color,
  const float scale
);

/// \brief Get the angle between two vector in CCW direction
/// \param v1 The first vector
/// \param v2 The second vector
/// \return The angle between two vector
double get_ccw_angle(const cv::Point & v1, const cv::Point v2);

/// \brief Find the index of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The index of the closesst tangram piece
size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour);

/// \brief Get the name of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The name of the closesst tangram piece
const std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour);

/// \brief Get the shape of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The shape of the closesst tangram piece
const std::vector<cv::Point> closest_tangram_piece_shape(const std::vector<cv::Point> & contour);

/// \brief Get the angle of rotation for a tangram piece
/// \param contour The contour of a tangram piece
/// \return The angle of rotation in radian
double get_tangram_piece_orientation(const std::vector<cv::Point> & contour, const cv::Mat & img);
} /// namespace tangram_utils

#endif /// TANGRAM_UTILS__TANGRAM_MATCH_HPP___
