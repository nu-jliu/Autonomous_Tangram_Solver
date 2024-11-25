#include <cmath>

#include "tangram_utils/tangram_match.hpp"

namespace tangram_utils
{
size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour)
{
  size_t best_match_index = -1;
  double best_match_score = DBL_MAX;

  for (size_t i = 0; i < tangrams_pieces.size(); ++i) {
    const auto piece = tangrams_pieces.at(i);
    const double match_score = cv::matchShapes(contour, piece, cv::CONTOURS_MATCH_I1, 0.0);

    std::cout << i << ": " << match_score << std::endl;

    if (match_score < best_match_score) {
      best_match_score = match_score;
      best_match_index = i;
    }
  }

  return best_match_index;
}

const std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour)
{
  size_t index = find_closest_tangram_piece(contour);

  if (index < 3) {
    const double area = cv::contourArea(contour);

    if (area < 4000.0) {
      /// Small Triangle
      index = 0;
    } else if (area < 12000.0) {
      /// Medium Triangle
      index = 1;
    } else {
      /// Large Triangle
      index = 2;
    }
  }

  const std::string name = tangram_names.at(index);
  return name;
}

const std::vector<cv::Point> closest_tangram_piece_shape(const std::vector<cv::Point> & contour)
{
  const size_t index = find_closest_tangram_piece(contour);
  const std::vector<cv::Point> shape = tangrams_pieces.at(index);

  return shape;
}

double tangram_piece_orientation(const std::vector<cv::Point> & contour)
{
  cv::RotatedRect ellipse = cv::fitEllipse(contour);
  const double angle = static_cast<double>(ellipse.angle);

  return angle;
}
} /// namespace tangram_utils
