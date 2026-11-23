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
     *        vector)
     * \param gap_in_meters Minimum distance between consecutive points
     *        (this defaults to 0.5 meter because carma by default creates a centerline with
     *        1 meter apart, so it should give just enough points to resample from)
     *
     * \return Filtered container of centerline points with minimum spacing enforced (same type as
     *         input)
     */

    template <typename PointContainer>
    PointContainer downsample_pts_with_min_meters(
    const PointContainer & original_centerline, double gap_in_meters = 0.5)
    {
        if (original_centerline.empty())
        {
            return original_centerline;
        }

        PointContainer filtered_centerline;

        // Always keep the first point
        filtered_centerline.push_back(original_centerline.front());

        for (size_t i = 1; i < original_centerline.size(); ++i)
        {
            // Get the last point we've added to our filtered centerline
            const auto & last_point = filtered_centerline.back();
            const auto & current_point = original_centerline[i];

            // Calculate squared distance between points (more efficient than using sqrt)
            double dx = current_point.x() - last_point.x();
            double dy = current_point.y() - last_point.y();
            double squared_distance = dx * dx + dy * dy;

            // Check if the squared distance is greater than or equal to the squared gap
            if (squared_distance >= gap_in_meters * gap_in_meters)
            {
                filtered_centerline.push_back(current_point);
            }
            // If distance is less than the gap, we skip this point
        }

        return filtered_centerline;
    }

    /**
     * \brief Downsamples the points by keeping every nth point, while always keeping the first and the last point.
     *
     * Plain downsampling (keeping indices 0, n, 2n, ...) drops up to n-1 points at the end of the input, so the
     * geometry stops short of the true end (e.g. a lanelet's end) and the dropped tail gets replaced by a chord.
     * Here the last point is always kept. If it falls closer than half of the last regular gap to the previously
     * kept point, it replaces that point instead of being appended, so that no very short segment is created
     * (which can destabilize the spline fit and curvature computation).
     *
     * \param points Container of points (BasicLineString2d or vector of BasicPoint2d)
     * \param n Keep every nth point. 0 returns an empty container, same as carma_ros2_utils downsample_vector
     *
     * \return Downsampled container including the first and last points of the input
     */
    template <typename PointContainer>
    PointContainer downsample_keep_endpoints(const PointContainer & points, unsigned int n)
    {
        PointContainer output = carma_ros2_utils::containers::downsample_vector(points, n);

        // First point is always kept by downsample_vector (index 0). Only the last point needs handling.
        if (output.empty() || points.size() < 2)
        {
            return output;
        }

        const auto & last_point = points.back();
        double dist_to_last = (last_point - output.back()).norm();

        if (dist_to_last < 1e-6)
        {
            return output; // Last point already included
        }

        if (output.size() > 1)
        {
            double last_gap = (output.back() - output[output.size() - 2]).norm();
            if (dist_to_last < 0.5 * last_gap)
            {
                output.back() = last_point; // Replace to avoid a very short final segment
                return output;
            }
        }

        output.push_back(last_point);
        return output;
    }

} // namespace waypoint_generation
} // namespace basic_autonomy
