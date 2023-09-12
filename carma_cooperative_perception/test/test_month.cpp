// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <carma_cooperative_perception/month.hpp>

#include <array>
#include <string>

TEST(Month, UnaryIncrement)
{
  carma_cooperative_perception::Month month{1};
  EXPECT_EQ(month++, carma_cooperative_perception::January);
  EXPECT_EQ(month++, carma_cooperative_perception::February);
  EXPECT_EQ(month++, carma_cooperative_perception::March);
  EXPECT_EQ(month++, carma_cooperative_perception::April);
  EXPECT_EQ(month++, carma_cooperative_perception::May);
  EXPECT_EQ(month++, carma_cooperative_perception::June);
  EXPECT_EQ(month++, carma_cooperative_perception::July);
  EXPECT_EQ(month++, carma_cooperative_perception::August);
  EXPECT_EQ(month++, carma_cooperative_perception::September);
  EXPECT_EQ(month++, carma_cooperative_perception::October);
  EXPECT_EQ(month++, carma_cooperative_perception::November);
  EXPECT_EQ(month++, carma_cooperative_perception::December);
  EXPECT_EQ(month, carma_cooperative_perception::January);

  EXPECT_EQ(++month, carma_cooperative_perception::February);
  EXPECT_EQ(++month, carma_cooperative_perception::March);
  EXPECT_EQ(++month, carma_cooperative_perception::April);
  EXPECT_EQ(++month, carma_cooperative_perception::May);
  EXPECT_EQ(++month, carma_cooperative_perception::June);
  EXPECT_EQ(++month, carma_cooperative_perception::July);
  EXPECT_EQ(++month, carma_cooperative_perception::August);
  EXPECT_EQ(++month, carma_cooperative_perception::September);
  EXPECT_EQ(++month, carma_cooperative_perception::October);
  EXPECT_EQ(++month, carma_cooperative_perception::November);
  EXPECT_EQ(++month, carma_cooperative_perception::December);
  EXPECT_EQ(++month, carma_cooperative_perception::January);
}

TEST(Month, UnaryDecrement)
{
  carma_cooperative_perception::Month month{12};
  EXPECT_EQ(month--, carma_cooperative_perception::December);
  EXPECT_EQ(month--, carma_cooperative_perception::November);
  EXPECT_EQ(month--, carma_cooperative_perception::October);
  EXPECT_EQ(month--, carma_cooperative_perception::September);
  EXPECT_EQ(month--, carma_cooperative_perception::August);
  EXPECT_EQ(month--, carma_cooperative_perception::July);
  EXPECT_EQ(month--, carma_cooperative_perception::June);
  EXPECT_EQ(month--, carma_cooperative_perception::May);
  EXPECT_EQ(month--, carma_cooperative_perception::April);
  EXPECT_EQ(month--, carma_cooperative_perception::March);
  EXPECT_EQ(month--, carma_cooperative_perception::February);
  EXPECT_EQ(month--, carma_cooperative_perception::January);
  EXPECT_EQ(month, carma_cooperative_perception::December);

  EXPECT_EQ(--month, carma_cooperative_perception::November);
  EXPECT_EQ(--month, carma_cooperative_perception::October);
  EXPECT_EQ(--month, carma_cooperative_perception::September);
  EXPECT_EQ(--month, carma_cooperative_perception::August);
  EXPECT_EQ(--month, carma_cooperative_perception::July);
  EXPECT_EQ(--month, carma_cooperative_perception::June);
  EXPECT_EQ(--month, carma_cooperative_perception::May);
  EXPECT_EQ(--month, carma_cooperative_perception::April);
  EXPECT_EQ(--month, carma_cooperative_perception::March);
  EXPECT_EQ(--month, carma_cooperative_perception::February);
  EXPECT_EQ(--month, carma_cooperative_perception::January);
  EXPECT_EQ(--month, carma_cooperative_perception::December);
}

TEST(Month, UnsignedConversion)
{
  const carma_cooperative_perception::Month month{5};
  EXPECT_EQ(static_cast<unsigned>(month), 5U);
}

TEST(Month, OkFunction)
{
  {
    const carma_cooperative_perception::Month month{0};
    EXPECT_FALSE(month.ok());
  }
  {
    const carma_cooperative_perception::Month month{1};
    EXPECT_TRUE(month.ok());
  }
  {
    const carma_cooperative_perception::Month month{12};
    EXPECT_TRUE(month.ok());
  }
  {
    const carma_cooperative_perception::Month month{13};
    EXPECT_FALSE(month.ok());
  }
}

TEST(Month, Equality)
{
  const carma_cooperative_perception::Month month{9};

  EXPECT_EQ(month, carma_cooperative_perception::September);
}

TEST(Month, NotEquality)
{
  const carma_cooperative_perception::Month month{19};

  EXPECT_NE(month, carma_cooperative_perception::September);
}

TEST(Month, LessThan)
{
  const carma_cooperative_perception::Month month{3};

  EXPECT_LT(month, carma_cooperative_perception::December);
}

TEST(Month, LessThanEqual)
{
  const carma_cooperative_perception::Month month{3};

  EXPECT_LE(month, carma_cooperative_perception::March);
}

TEST(Month, GreaterThan)
{
  const carma_cooperative_perception::Month month{19};

  EXPECT_GT(month, carma_cooperative_perception::September);
}

TEST(Month, GreaterThanEqual)
{
  const carma_cooperative_perception::Month month{10};

  EXPECT_GE(month, carma_cooperative_perception::October);
}

TEST(MonthFromNumber, Ostream)
{
  static constexpr std::array abbreviations{"Jan.", "Feb.", "Mar.", "Apr.", "May",  "Jun.",
                                            "Jul.", "Aug.", "Sep.", "Oct.", "Nov.", "Dec."};

  {
    std::stringstream stream;
    stream << carma_cooperative_perception::Month{static_cast<std::uint8_t>(0)};
    EXPECT_EQ(stream.str(), "0 is not a valid month");
  }

  for (auto m{1}; m <= 12; ++m) {
    std::stringstream stream;
    stream << carma_cooperative_perception::Month{static_cast<std::uint8_t>(m)};
    EXPECT_EQ(stream.str(), abbreviations.at(m - 1));
  }

  {
    std::stringstream stream;
    stream << carma_cooperative_perception::Month{static_cast<std::uint8_t>(13)};
    EXPECT_EQ(stream.str(), "13 is not a valid month");
  }
}
