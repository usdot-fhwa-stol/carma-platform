#pragma once

#include <vector>
#include <carma_wm/Geometry.h>
#include <inlanecruising_plugin/third_party_library/spline.h>
#include <inlanecruising_plugin/smoothing/SplineI.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
class CubicSpline : public SplineI
{
public:
  ~CubicSpline(){};
  void setPoints(std::vector<lanelet::BasicPoint2d> points) override;
  double operator()(double x) const override;

private:
  tk::spline spline_;
};
};  // namespace smoothing
};  // namespace inlanecruising_plugin