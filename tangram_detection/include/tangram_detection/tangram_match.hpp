#ifndef TANGRAM_DETECTION__TANGRAM_MATCH_HPP___
#define TANGRAM_DETECTION__TANGRAM_MATCH_HPP___

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

namespace tangram_detection
{
// Define the tangrams as a constant vector
const std::vector<std::vector<cv::Point>> tangrams_pieces{
  // Small triangle
  {cv::Point(0, 0), cv::Point(1, 0), cv::Point(0, 1)},

  // Medium triangle
  {cv::Point(0, 0), cv::Point(2, 0), cv::Point(0, 2)},

  // Large triangle
  {cv::Point(0, 0), cv::Point(4, 0), cv::Point(0, 4)},

  // Square
  {cv::Point(0, 0), cv::Point(2, 0), cv::Point(2, 2), cv::Point(0, 2)},

  // Parallelogram
  {cv::Point(0, 0), cv::Point(35, 0), cv::Point(23, 23), cv::Point(60, 23)}
};

const std::vector<std::string> tangram_names{
  "Small Triangle",
  "Medium Triangle",
  "Large Triangle",
  "Square",
  "Parallogram"
};

size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour);

std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour);

std::vector<cv::Point> closest_tangram_piece_shape(const std::vector<cv::Point> & contour);
}

#endif
