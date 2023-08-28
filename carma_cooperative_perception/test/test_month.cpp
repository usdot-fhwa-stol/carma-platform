#include <gtest/gtest.h>

#include <carma_cooperative_perception/msg_conversion.hpp>

TEST(MonthFromNumber, Normal)
{
  const auto month{carma_cooperative_perception::month_from_number(1)};

  EXPECT_EQ(month, carma_cooperative_perception::Month::kJanuary);
}
