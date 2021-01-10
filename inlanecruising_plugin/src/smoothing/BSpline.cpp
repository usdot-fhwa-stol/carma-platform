#include <inlanecruising_plugin/smoothing/BSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXf matrix_points(2, points.size());
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points, 2);
}
Eigen::VectorXf BSpline::operator()(double t) const
{
  Eigen::VectorXf values = spline_(t);
  return values;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin