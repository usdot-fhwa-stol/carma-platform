#include <inlanecruising_plugin/smoothing/BSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXd matrix_points(2, points.size());
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(matrix_points, 2);
}
lanelet::BasicPoint2d BSpline::operator()(double t) const
{
  Eigen::VectorXd values = spline_(t);
  lanelet::BasicPoint2d pt = {values.x(), values.y()};
  return pt;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin