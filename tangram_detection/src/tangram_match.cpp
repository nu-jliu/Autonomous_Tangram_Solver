#include "tangram_detection/tangram_match.hpp"

namespace tangram_detection
{
size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour)
{
  size_t best_match_index = -1;
  double best_match_score = DBL_MAX;

  for (size_t i = 0; i < tangrams_pieces.size(); ++i) {
    const auto piece = tangrams_pieces.at(i);
    const double match_score = cv::matchShapes(contour, piece, cv::CONTOURS_MATCH_I1, 0.0);

    if (match_score < best_match_score) {
      best_match_score = match_score;
      best_match_index = i;
    }
  }

  return best_match_index;
}

std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour)
{
  const size_t index = find_closest_tangram_piece(contour);
  const std::string name = tangram_names.at(index);

  return name;
}

std::vector<cv::Point> closest_tangram_piece_shape(const std::vector<cv::Point> & contour)
{
  const size_t index = find_closest_tangram_piece(contour);
  const std::vector<cv::Point> shape = tangrams_pieces.at(index);

  return shape;
}
}
