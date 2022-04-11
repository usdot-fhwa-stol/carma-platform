#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <carma_perception_msgs/msg/predicted_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <tuple>

namespace object {

namespace conversions {
namespace impl {
/**
 * \brief Composes a PredictedState message form a provided current point and previous point. It
 * calculates the speed from these points using mobility_path_time_step
 * \param curr_pt current point
 * \param prev_pt prev_pt. this point is recorded in the state
 * \param prev_time_stamp prev_pt's time stamp. This time is recorded in the state
 * \param prev_yaw A previous yaw value in radians to use if the two provided points are equivalent
 * \return std::pair<carma_perception_msgs::msg::PredictedState,double> where the first element is the prediction
 * including linear velocity, last_time, orientation filled in and the second element is the yaw in radians used to
 * compute the orientation
 */
std::pair<carma_perception_msgs::msg::PredictedState, double> composePredictedState(const tf2::Vector3 &curr_pt,
                                                                                    const tf2::Vector3 &prev_pt,
                                                                                    const rclcpp::Time &prev_time_stamp,
                                                                                    double prev_yaw);
/**
 * \brief Helper function to fill in the angular velocity of the external object
 * \param ExternalObject to fill in its angular velocities
 */
void calculateAngVelocityOfPredictedStates(carma_perception_msgs::msg::ExternalObject &object);

double getYawFromQuaternionMsg(const geometry_msgs::msg::Quaternion &quaternion);

/**
 * \brief Transforms ecef point to map frame using internally saved map transform
 * \param ecef_point ecef_point to transform
 * \param map_projector Projector used for frame conversion
 * \return point in map
 */
tf2::Vector3 transform_to_map_frame(const tf2::Vector3 &ecef_point, const lanelet::projection::LocalFrameProjector &map_projector);

/**
 * \brief Converts a lat/lon GNSS point to map frame using the provided projector
 * \param lat The latitude in degrees
 * \param lon The longitude in degrees
 * \param ele The elevation in degrees
 * \return The 3d point in the map frame
 */ 
tf2::Vector3 gnss_to_map(double lat, double lon, double ele, const lanelet::projection::LocalFrameProjector &map_projector);
}  // namespace impl
}  // namespace conversions
}  // namespace object
