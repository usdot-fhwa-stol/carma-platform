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

#ifndef CARMA_COOPERATIVE_PERCEPTION__MONTH_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__MONTH_HPP_

/**
 * This file contains a Month class implementation that should be source-compatible
 * with std::chrono::month. Until CARMA targets C++20, we will have to use this
 * instead of the standard library.
*/

#include <array>
#include <ostream>

namespace carma_cooperative_perception
{
class Month
{
public:
  Month() = default;

  /**
   * @brief Create a Month instance with the specified value
   *
   * @param[in] month_value Numerical value of the month; typically between [1, 12]
  */
  constexpr explicit Month(std::uint8_t month_value) noexcept : month_value_{month_value} {}

  /**
   * @brief Pre-increment operator overload
   *
   * @return Reference to Month instance being incremented
  */
  constexpr auto operator++() noexcept -> Month &
  {
    constexpr auto jan_value{1U};
    constexpr auto dec_value{12U};

    month_value_ = (month_value_ + 1) % (dec_value + 1);

    if (month_value_ == 0) {
      month_value_ = jan_value;
    }

    return *this;
  }

  /**
   * @brief Post-increment operator overload
   *
   * @param[in] _ Dummy parameter used to distinguish from the pre-increment operator
   *
   * @return Copy of Month instance before it was incremented
  */
  constexpr auto operator++(int /* dummy parameter */) noexcept -> Month
  {
    Month previous{*this};
    ++(*this);

    return previous;
  }

  /**
   * @brief Pre-decrement operator overload
   *
   * @return Reference to Month instance being decremented
  */
  constexpr auto operator--() noexcept -> Month &
  {
    constexpr auto dec_value{12U};

    month_value_ = (month_value_ - 1 % (dec_value + 1));

    if (month_value_ == 0) {
      month_value_ = dec_value;
    }

    return *this;
  }

  /**
   * @brief Post-decrement operator overload
   *
   * @param[in] _ Dummy parameter used to distinguish from the pre-decrement operator
   *
   * @return Copy of Month instance before it was decremented
  */
  constexpr auto operator--(int /* dummy parameter */) noexcept -> Month
  {
    Month previous{*this};
    --(*this);

    return previous;
  }

  /**
   * @brief Conversion function to convert Month instance to unsigned type
  */
  constexpr explicit operator unsigned() const noexcept
  {
    return static_cast<unsigned>(month_value_);
  }

  /**
   * @brief Checks if Month instance's value is within valid Gregorian calendar range
   *
   * @return true if Month instance's value is within [1, 12]; false otherwise
  */
  [[nodiscard]] constexpr auto ok() const noexcept -> bool
  {
    constexpr auto jan_value{1U};
    constexpr auto dec_value{12U};

    return month_value_ >= jan_value && month_value_ <= dec_value;
  }

  /**
   * @brief Compare exact equality between two Month instances
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is Month instances are equal; false otherwise
  */
  friend constexpr auto operator==(const Month & x, const Month & y) noexcept -> bool
  {
    return x.month_value_ == y.month_value_;
  }

  /**
   * @brief Compare exact inequality between two Month instances
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is Month instances are not equal; false otherwise
  */
  friend constexpr auto operator!=(const Month & x, const Month & y) noexcept -> bool
  {
    return !(x == y);
  }

  /**
   * @brief Check if one Month instance is less than another
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is x comes before y in the calendar; false otherwise
  */
  friend constexpr auto operator<(const Month & x, const Month & y) noexcept -> bool
  {
    return x.month_value_ < y.month_value_;
  }

  /**
   * @brief Check if one Month instance is less than or equal to another
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is x comes before y in the calendar or if instances are equal; false otherwise
  */
  friend constexpr auto operator<=(const Month & x, const Month & y) noexcept -> bool
  {
    return x < y || x == y;
  }

  /**
   * @brief Check if one Month instance is greater than another
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is x comes after y in the calendar; false otherwise
  */
  friend constexpr auto operator>(const Month & x, const Month & y) noexcept -> bool
  {
    return x.month_value_ > y.month_value_;
  }

  /**
   * @brief Check if one Month instance is greater than or equal to another
   *
   * @param[in] x First Month instance
   * @param[in] y Second Month instance
   *
   * @return true is x comes after y in the calendar or if instances are equal; false otherwise
  */
  friend constexpr auto operator>=(const Month & x, const Month & y) noexcept -> bool
  {
    return x > y || x == y;
  }

  /**
   * @brief Output a string representation of Month instance to an output stream
   *
   * @tparam CharT Character type of the specified stream
   * @tparam Traits Character traits of the specified stream
   *
   * @param[in,out] os Output stream being written to
   * @param[in] m Month instance being written out
   *
   * @return Reference to specified output stream
  */
  template <typename CharT, typename Traits>
  friend auto operator<<(std::basic_ostream<CharT, Traits> & os, const Month & m)
    -> std::basic_ostream<CharT, Traits> &
  {
    static constexpr std::array abbreviations{"Jan.", "Feb.", "Mar.", "Apr.", "May",  "Jun.",
                                              "Jul.", "Aug.", "Sep.", "Oct.", "Nov.", "Dec."};
    if (!m.ok()) {
      return os << static_cast<unsigned>(m) << " is not a valid month";
    }

    return os << abbreviations.at(static_cast<unsigned>(m) - 1);
  }

private:
  std::uint8_t month_value_;
};

inline constexpr Month January{1};
inline constexpr Month February{2};
inline constexpr Month March{3};
inline constexpr Month April{4};
inline constexpr Month May{5};
inline constexpr Month June{6};
inline constexpr Month July{7};
inline constexpr Month August{8};
inline constexpr Month September{9};
inline constexpr Month October{10};
inline constexpr Month November{11};
inline constexpr Month December{12};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__MONTH_HPP_
