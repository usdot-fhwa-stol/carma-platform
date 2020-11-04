#pragma once

#include <vector>
#include <carma_wm/Geometry.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
class SplineI
{
public:
  /**
   * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
   *
   */
  virtual ~SplineI(){};
  virtual void setPoints(std::vector<lanelet::BasicPoint2d> points) = 0;
  virtual double operator()(double x) const = 0;
};
};  // namespace smoothing
};  // namespace inlanecruising_plugin