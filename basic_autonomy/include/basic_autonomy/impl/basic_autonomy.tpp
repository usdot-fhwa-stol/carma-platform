#include <basic_autonomy/basic_autonomy.hpp>

namespace basic_autonomy
{
namespace waypoint_generation
{
    /**
     * \brief Filters centerline points to ensure consecutive points are at least the specified distance
     * apart
     *
     * This template function takes either a BasicLineString2d or a vector of centerline points and
     * returns a filtered version of the same type where no two consecutive points are closer than the
     * specified minimum gap. This helps reduce redundant points while maintaining the overall shape of
     * the centerline.
     *
     * \param original_centerline Container of centerline points to be filtered (BasicLineString2d or
     * vector) \param gap_in_meters Minimum distance between consecutive points
     *          (this defaults to 0.5 meter because carma by default creates a centerline with
     *          1 meter apart, so it should give just enough points to resample from)
     *
     * \return Filtered container of centerline points with minimum spacing enforced (same type as
     * input)
     */

    template <typename PointContainer>
    PointContainer downsample_pts_with_min_meters(
    const PointContainer & original_centerline, double gap_in_meters = 0.5)
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