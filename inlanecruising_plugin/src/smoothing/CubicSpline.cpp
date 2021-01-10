#include <inlanecruising_plugin/smoothing/CubicSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void CubicSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  std::vector<double> x;
  std::vector<double> y;
  for (const auto& p : points)
  {
    x.push_back(p.x());
    y.push_back(p.y());
  }
  spline_.set_points(x, y, true);
}
double CubicSpline::operator()(double x) const
{
  return spline_(x);
}
Eigen::VectorXf CubicSpline::operator[](double x) const
{
  Eigen::Vector2f output = {x, spline_(x)};
  return output;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin