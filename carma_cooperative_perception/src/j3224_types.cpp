#include "carma_cooperative_perception/j3224_types.hpp"

namespace carma_cooperative_perception
{

auto PositionOffsetXYZ::from_msg(const j3224_v2x_msgs::msg::PositionOffsetXYZ & msg) noexcept
  -> PositionOffsetXYZ
{
  PositionOffsetXYZ offset{
    .offset_x{units::length::decimeter_t{static_cast<double>(msg.offset_x.object_distance)}},
    .offset_y{units::length::decimeter_t{static_cast<double>(msg.offset_y.object_distance)}},
    .offset_z{std::nullopt}};

  if (msg.presence_vector & msg.HAS_OFFSET_Z) {
    offset.offset_z = units::length::decimeter_t{static_cast<double>(msg.offset_z.object_distance)};
  }

  return offset;
}

auto MeasurementTimeOffset::from_msg(
  const j3224_v2x_msgs::msg::MeasurementTimeOffset & msg) noexcept -> MeasurementTimeOffset
{
  return MeasurementTimeOffset{
    units::time::millisecond_t{static_cast<double>(msg.measurement_time_offset)}};
}

}  // namespace carma_cooperative_perception
