#ifndef PUZZLE_SOLVER__TANGRAM_MATCH_HPP___
#define PUZZLE_SOLVER__TANGRAM_MATCH_HPP___

#include <opencv2/opencv.hpp>
#include <vector>

namespace puzzle_solver
{
/// \brief The shape of all tangram pieces
const std::vector<std::vector<cv::Point>> tangrams_pieces{
  // Small triangle
  {cv::Point(0, 0), cv::Point(23, 0), cv::Point(0, 23)},

  // Medium triangle
  {cv::Point(0, 0), cv::Point(37, 0), cv::Point(0, 37)},

  // Large triangle
  {cv::Point(0, 0), cv::Point(63, 0), cv::Point(0, 63)},

  // Square
  {cv::Point(0, 0), cv::Point(35, 0), cv::Point(35, 35), cv::Point(0, 35)},

  // Parallelogram
  {cv::Point(0, 0), cv::Point(37, 0), cv::Point(60, 23), cv::Point(23, 23)}
};

/// @brief
const std::vector<std::string> tangram_names{
  "Small Triangle",
  "Medium Triangle",
  "Large Triangle",
  "Square",
  "Parallelogram"
};

/// \brief Find the index of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The index of the closesst tangram piece
size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour);

/// \brief Get the name of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The name of the closesst tangram piece
std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour);

/// \brief Get the shape of the closest matching tangram piece
/// \param contour The contour of the detected tangram piece
/// \return The shape of the closesst tangram piece
std::vector<cv::Point> closest_tangram_piece_shape(const std::vector<cv::Point> & contour);
} // namespace puzzle_solver

#endif // PUZZLE_SOLVER__TANGRAM_MATCH_HPP___
