#pragma once

#include <vector>
#include <deque>
namespace inlanecruising_plugin
{
namespace smoothing
{
std::vector<double> moving_average_filter(const std::vector<double> input, int window_size)
{
  int i = 0;
  double average;
  std::deque<double> samples;
  std::vector<double> output;
  output.reserve(input.size());
  for (auto value : input)
  {
    if (i < window_size)
    {
      samples.push_back(value);
    }
    else
    {
      samples.pop_front();
      samples.push_back(value);
    }

    double total = 0;
    for (auto s : samples)
    {
      total += s;
    }

    double average = total / samples.size();
    output.push_back(average);
    i++;
  }

  return output;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin