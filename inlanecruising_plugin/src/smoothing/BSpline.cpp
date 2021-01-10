#include <inlanecruising_plugin/smoothing/BSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXf matrix_points(3, points.size());
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << 2, point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline3d>::Interpolate(matrix_points, 2);
}
double BSpline::operator()(double x) const
{
  Eigen::VectorXf values = spline_(x);
  return values.z();
}
Eigen::VectorXf BSpline::operator[](double x) const
{
  Eigen::VectorXf values = spline_(x);
  return values;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin