#include <cmath>

#include "tangram_utils/tangram_match.hpp"

namespace tangram_utils
{
void draw_axis(
  const cv::Mat & img,
  cv::Point p,
  cv::Point q,
  const cv::Scalar & color,
  const float scale = 0.2
)
{
  const double angle = atan2( (double) p.y - q.y, (double) p.x - q.x); // angle in radians
  const double hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

  // Here we lengthen the arrow by a factor of scale
  q.x = (int) (p.x - scale * hypotenuse * std::cos(angle));
  q.y = (int) (p.y - scale * hypotenuse * std::sin(angle));
  cv::line(img, p, q, color, 1, cv::LINE_AA);

  // create the arrow hooks
  p.x = (int) (q.x + 9 * std::cos(angle + CV_PI / 4));
  p.y = (int) (q.y + 9 * std::sin(angle + CV_PI / 4));
  cv::line(img, p, q, color, 1, cv::LINE_AA);

  p.x = (int) (q.x + 9 * std::cos(angle - CV_PI / 4));
  p.y = (int) (q.y + 9 * std::sin(angle - CV_PI / 4));
  cv::line(img, p, q, color, 1, cv::LINE_AA);
}

bool validate_pieces(const std::vector<size_t> & shapes)
{
  int num_st = 0; /// Small Triangle
  int num_mt = 0; /// Medium Triangle
  int num_lt = 0; /// Large Triangle
  int num_sq = 0; /// Square
  int num_pl = 0; /// Parallelogram

  for (const auto shape : shapes) {
    switch (shape) {
      case 0:
        ++num_st;
        break;

      case 1:
        ++num_mt;
        break;

      case 2:
        ++num_lt;
        break;

      case 3:
        ++num_sq;
        break;

      case 4:
        ++num_pl;
        break;

      default:
        return false;
    }
  }

  if (num_lt != 2) {
    return false;
  } else if (num_mt != 1) {
    return false;
  } else if (num_st != 2) {
    return false;
  } else if (num_sq != 1) {
    return false;
  } else if (num_pl != 1) {
    return false;
  }

  return true;
}

double get_ccw_angle(const cv::Point2d & v1, const cv::Point2d & v2)
{
  const double dot_product = v1.dot(v2);
  const double cross_product = v1.cross(v2);

  const double angle = std::atan2(cross_product, dot_product);
  return angle;
}

bool is_triangle_flipped(const std::vector<cv::Point> & contour)
{
  // std::cout << "Size: " << static_cast<int>(contour.size()) << std::endl;

  double x_total = 0;
  double y_total = 0;

  for (const auto & point : contour) {
    x_total += static_cast<double>(point.x);
    y_total += static_cast<double>(point.y);
  }

  const int x_avg = static_cast<int>(x_total / static_cast<double>(contour.size()));
  const int y_avg = static_cast<int>(y_total / static_cast<double>(contour.size()));

  (void) x_avg;

  const cv::Point center = find_center(contour);

  return y_avg > center.y;
}

const cv::Point find_center(const std::vector<cv::Point> & contour)
{
  const cv::Moments m = cv::moments(contour);

  if (m.m00 != 0) {
    const int cx = static_cast<int>(m.m10 / m.m00);
    const int cy = static_cast<int>(m.m01 / m.m00);

    return cv::Point(cx, cy);
  } else {
    throw std::runtime_error("Invalid moment");
  }
}

size_t find_closest_tangram_piece(const std::vector<cv::Point> & contour)
{
  size_t best_match_index = -1;
  double best_match_score = DBL_MAX;

  for (size_t i = 0; i < tangrams_pieces.size(); ++i) {
    const auto piece = tangrams_pieces.at(i);
    const double match_score = cv::matchShapes(contour, piece, cv::CONTOURS_MATCH_I1, 0.0);

    // std::cout << i << ": " << match_score << std::endl;

    if (match_score < best_match_score) {
      best_match_score = match_score;
      best_match_index = i;
    }
  }

  if (best_match_index < 3) {
    const double area = cv::contourArea(contour);

    if (area < small_medium_triangle_threshold) {
      /// Small Triangle
      best_match_index = 0;
    } else if (area < medium_large_triangle_threshold) {
      /// Medium Triangle
      best_match_index = 1;
    } else {
      /// Large Triangle
      best_match_index = 2;
    }
  }

  return best_match_index;
}

const std::string closest_tangram_piece_name(const std::vector<cv::Point> & contour)
{
  const size_t index = find_closest_tangram_piece(contour);
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
  const cv::Moments mu = cv::moments(contour);

  if (mu.mu20 == mu.mu02) {
    return -1;
  }

  const double radian = 0.5 * std::atan2(2 * mu.mu11, mu.mu20 - mu.mu02);
  return radian;
}

double get_tangram_piece_orientation(const std::vector<cv::Point> & contour, const cv::Mat & img)
{
  const int sz = static_cast<int>(contour.size());
  cv::Mat data_pts(sz, 2, CV_64F);
  for (int i = 0; i < data_pts.rows; i++) {
    data_pts.at<double>(i, 0) = contour.at(i).x;
    data_pts.at<double>(i, 1) = contour.at(i).y;
  }

  //Perform PCA analysis
  const cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

  //Store the center of the object
  cv::Point center(
    static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
    static_cast<int>(pca_analysis.mean.at<double>(0, 1))
  );

  //Store the eigenvalues and eigenvectors
  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i) {
    eigen_vecs.at(i) = cv::Point2d(
      pca_analysis.eigenvectors.at<double>(i, 0),
      pca_analysis.eigenvectors.at<double>(i, 1));

    eigen_val.at(i) = pca_analysis.eigenvalues.at<double>(i);
  }

  const cv::Point2d v1 = eigen_vecs.at(0);
  const cv::Point2d v2 = eigen_vecs.at(1);
  const double angle_ccw = get_ccw_angle(v1, v2);

  if (angle_ccw > CV_PI) {
    eigen_vecs.at(0).x *= -1.0;
    eigen_vecs.at(0).y *= -1.0;
  }

  // Draw the principal components
  cv::Point p1 = center + 0.02 * cv::Point(
    static_cast<int>(eigen_vecs.at(0).x * eigen_val.at(0)),
    static_cast<int>(eigen_vecs.at(0).y * eigen_val.at(0))
  );
  cv::Point p2 = center - 0.02 * cv::Point(
    static_cast<int>(eigen_vecs.at(1).x * eigen_val.at(1)),
    static_cast<int>(eigen_vecs.at(1).y * eigen_val.at(1))
  );
  draw_axis(img, center, p1, cv::Scalar(0, 0, 255), 1);
  draw_axis(img, center, p2, cv::Scalar(0, 255, 0), 1);

  const cv::Point2d principal_axis = eigen_vecs.at(1);
  const double angle = atan2(principal_axis.y, principal_axis.x);   // orientation in radians

  return angle;
}
} /// namespace tangram_utils
