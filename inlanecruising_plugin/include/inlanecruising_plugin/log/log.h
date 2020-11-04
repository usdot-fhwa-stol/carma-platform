#include <sstream>
#include <inlanecruising_plugin/inlanecruising_plugin.h>

namespace inlanecruising_plugin
{
namespace log
{
std::string basicPointToStream(lanelet::BasicPoint2d point)
{
  std::ostringstream out;
  out << point.x() << ", " << point.y();
  return out.str();
}

std::string pointSpeedPairToStream(PointSpeedPair point)
{
  std::ostringstream out;
  out << "Point: " << basicPointToStream(point.point) << " Speed: " << point.speed;
  return out.str();
}

template <class T>
void printDebugPerLine(const std::vector<T>& values, std::function<std::string(T)> func)
{
  for (const auto& value : values)
  {
    ROS_DEBUG_STREAM(func(value));
  }
}

template <class T>
void printDebugPerLine(const std::vector<T>& values, std::string (*free_func)(T))
{
  auto function = static_cast<std::function<std::string(T)>>(free_func);
  printDebugPerLine(values, function);
}

void printDoublesPerLineWithPrefix(const std::string& prefix, const std::vector<double>& values)
{
  for (const auto& value : values)
  {
    ROS_DEBUG_STREAM(prefix << value);
  }
}

}  // namespace log
}  // namespace inlanecruising_plugin