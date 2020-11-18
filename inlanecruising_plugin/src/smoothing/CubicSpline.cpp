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

double CubicSpline::first_deriv(double x) const {
  return spline_.deriv(1, x);
}

double CubicSpline::second_deriv(double x) const {
  return spline_.deriv(2, x);
}

};  // namespace smoothing
};  // namespace inlanecruising_plugin