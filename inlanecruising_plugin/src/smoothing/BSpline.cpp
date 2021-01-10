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

Eigen::Vector3d BSpline::first_deriv(double t) const {
  Eigen::Array2Xf v = spline_.derivatives(t, 1);
  Eigen::Vector3d output = {v(2), v(3), 0};
  return output;
}

Eigen::Vector3d BSpline::second_deriv(double t) const {
  Eigen::Array2Xf v = spline_.derivatives(t, 2);
  Eigen::Vector3d output = {v(4), v(5), 0};
  return output;
}

};  // namespace smoothing
};  // namespace inlanecruising_plugin