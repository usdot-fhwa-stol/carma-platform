#include <basic_autonomy/basic_autonomy.hpp>

namespace basic_autonomy
{
namespace waypoint_generation
{
    template <typename PointContainer>
    PointContainer downsample_pts_with_min_meters(
    const PointContainer & original_centerline, double gap_in_meters)
    {
    if (original_centerline.empty()) {
        return original_centerline;
    }

    PointContainer filtered_centerline;

    // Always keep the first point
    filtered_centerline.push_back(original_centerline.front());

    for (size_t i = 1; i < original_centerline.size(); ++i) {
        // Get the last point we've added to our filtered centerline
        const auto & last_point = filtered_centerline.back();
        const auto & current_point = original_centerline[i];

        // Calculate squared distance between points (more efficient than using sqrt)
        double dx = current_point.x() - last_point.x();
        double dy = current_point.y() - last_point.y();
        double squared_distance = dx * dx + dy * dy;

        // Check if the squared distance is greater than or equal to the squared gap
        if (squared_distance >= gap_in_meters * gap_in_meters) {
        filtered_centerline.push_back(current_point);
        }
        // If distance is less than the gap, we skip this point
    }

    return filtered_centerline;
    }

} // namespace waypoint_generation
} // namespace basic_autonomy