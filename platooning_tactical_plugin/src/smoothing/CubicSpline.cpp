#include <platooning_tactical_plugin/smoothing/CubicSpline.h>

namespace platooning_tactical_plugin
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
};  // namespace smoothing
};  // namespace platooning_tactical_plugin
