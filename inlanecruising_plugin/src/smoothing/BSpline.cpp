#include <inlanecruising_plugin/smoothing/BSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void BSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  Eigen::MatrixXf matrix_points(3, points.size());
  std::cerr << "Inside SetPoints" << std::endl;
  for (auto pt: points)
  {
    std::cerr << "x:" << pt.x() << "y: " << pt.y() << std::endl;
  }
  int row_index = 0;
  for(auto const point : points){
      matrix_points.col(row_index) << 2, point.x(), point.y();
      row_index++;
  }
  spline_ = Eigen::SplineFitting<Spline3d>::Interpolate(matrix_points, 2);
  Eigen::VectorXf values = spline_(-71.8552);
  std::cerr << ">>>> CHECK2: x:-71.8552 << y:" << values.z() << std::endl;
  std::cerr << values << std::endl;
}
double BSpline::operator()(double x) const
{
  Eigen::VectorXf values = spline_(x);
  std::cerr << "Official: Operator:" << values << std::endl;
  return values.z();
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin