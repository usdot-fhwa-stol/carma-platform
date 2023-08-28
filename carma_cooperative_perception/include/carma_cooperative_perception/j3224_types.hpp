#ifndef CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_

#include <units.h>

#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>
#include <j3224_v2x_msgs/msg/position_offset_xyz.hpp>
#include <optional>

namespace carma_cooperative_perception
{

struct PositionOffsetXYZ
{
  units::length::decimeter_t offset_x;
  units::length::decimeter_t offset_y;
  std::optional<units::length::decimeter_t> offset_z;

  static auto from_msg(const j3224_v2x_msgs::msg::PositionOffsetXYZ & msg)
  {
    PositionOffsetXYZ offset{
      .offset_x{units::length::decimeter_t{static_cast<double>(msg.offset_x.object_distance)}},
      .offset_y{units::length::decimeter_t{static_cast<double>(msg.offset_y.object_distance)}},
      .offset_z{std::nullopt}};

    if (msg.presence_vector & msg.HAS_OFFSET_Z) {
      offset.offset_z =
        units::length::decimeter_t{static_cast<double>(msg.offset_z.object_distance)};
    }

    return offset;
  }
};

struct MeasurementTimeOffset
{
  units::time::millisecond_t measurement_time_offset;

  static auto from_msg(const j3224_v2x_msgs::msg::MeasurementTimeOffset & msg)
  {
    return MeasurementTimeOffset{
      units::time::millisecond_t{static_cast<double>(msg.measurement_time_offset)}};
  }
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_
