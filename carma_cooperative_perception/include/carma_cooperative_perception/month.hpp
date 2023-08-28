#ifndef CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_

namespace carma_cooperative_perception
{

// Should be replaced with std::chrono::month when CARMA targets C++20
enum class Month
{
  kJanuary = 1
};

static inline auto month_from_number(int month)
{
  switch (month) {
    case 1:
      return Month::kJanuary;
    default:
      return Month::kJanuary;
  }
}

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_
