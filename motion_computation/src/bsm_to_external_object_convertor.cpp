// Copyright 2019-2023 Leidos
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

#include <carma_perception_msgs/msg/external_object.hpp>
#include <motion_computation/impl/psm_to_external_object_helpers.hpp>
#include <motion_computation/message_conversions.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace motion_computation
{

namespace conversion
{

void convert(
  const carma_v2x_msgs::msg::BSM & in_msg, carma_perception_msgs::msg::ExternalObject & out_msg,
  const std::string & map_frame_id, double pred_period, double pred_step_size,
  const lanelet::projection::LocalFrameProjector & map_projector,
  tf2::Quaternion ned_in_map_rotation)
{
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;

  // assume any BSM is a dynamic object since its sent be a vehicle
  out_msg.dynamic_obj = true;
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::DYNAMIC_OBJ_PRESENCE;

  /////// Object Id /////////
  // Generate a unique object id from the bsm id
  out_msg.id = 0;
  for (int i = in_msg.core_data.id.size() - 1; i >= 0;
       i--) {  // using signed iterator to handle empty case
    // each byte of the bsm id gets placed in one byte of the object id.
    // This should result in very large numbers which will be unlikely to
    // conflict with standard detections
    out_msg.id |= in_msg.core_data.id[i] << (8 * i);
  }
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::ID_PRESENCE_VECTOR;

  /////// BSM Id /////////
  out_msg.bsm_id = in_msg.core_data.id;
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;

  /////// Timestamp /////////
  // Compute the timestamp
  out_msg.header.stamp = in_msg.header.stamp;
  out_msg.header.frame_id = map_frame_id;

  /////// Object Type and Size /////////

  if (
    in_msg.core_data.size.presence_vector &
    carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE) {
    // ExternalObject size is half of each dimension
    out_msg.size.x = in_msg.core_data.size.vehicle_length / 2;
  } else {
    out_msg.size.x = 1.0;  // value from mob path to external obj conversion
  }

  if (
    in_msg.core_data.size.presence_vector &
    carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE) {
    // ExternalObject size is half of each dimension
    out_msg.size.y = in_msg.core_data.size.vehicle_width / 2;
  } else {
    out_msg.size.y = 1.0;  // value from mob path to external obj conversion
  }

  out_msg.size.z = 2.0;  // value from mob path to external obj conversion

  out_msg.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;

  /////// Velocity /////////
  // Set the velocity
  if (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE) {
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::VELOCITY_PRESENCE_VECTOR;
    float_t speed_mps = in_msg.core_data.speed;

    out_msg.velocity.twist.linear.x = std::min(speed_mps, in_msg.core_data.SPEED_MAX);
  }

  // NOTE: The velocity covariance is not provided in the BSM. In order to
  // compute it you need at least two BSM messages
  //     Tracking and associating BSM messages would be an increase in
  //     complexity for this conversion which is not warranted without an
  //     existing use case for the velocity covariance. If a use case is
  //     presented for it, such an addition can be made at that time.

  /////// Confidences /////////
  // Compute the position covariance
  // For computing confidence we will use the largest provided standard
  // deviation of position
  double largest_position_std =
    std::max(in_msg.core_data.accuracy.semi_major, in_msg.core_data.accuracy.semi_minor);

  double lat_variance = in_msg.core_data.accuracy.semi_minor * in_msg.core_data.accuracy.semi_minor;

  double lon_variance = in_msg.core_data.accuracy.semi_major * in_msg.core_data.accuracy.semi_major;

  double heading_variance =
    in_msg.core_data.accuracy.orientation * in_msg.core_data.accuracy.orientation;

  double position_confidence = 0.1;  // Default will be 10% confidence. If the position accuracy is
                                     // available then this value will be updated

  // A standard deviation which is larger than the acceptable value to give
  // 95% confidence interval on fitting the pedestrian within one 3.7m lane
  constexpr double MAX_POSITION_STD = 1.85;

  if (
    (in_msg.core_data.accuracy.presence_vector |
     carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) &&
    (in_msg.core_data.accuracy.presence_vector |
     carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE)) {
    // Both accuracies available

    // NOTE: ExternalObject.msg does not clearly define what is meant by
    // position confidence
    //     Here we are providing a linear scale based on the positional accuracy
    //     where 0 confidence would denote A standard deviation which is larger
    //     than the acceptable value to give 95% confidence interval on fitting
    //     the pedestrian within one 3.7m lane
    // Set the confidence
    // Without a way of getting the velocity confidence from the BSM we will use
    // the position confidence for both
    out_msg.confidence = 1.0 - std::min(1.0, fabs(largest_position_std / MAX_POSITION_STD));
    out_msg.presence_vector |=
      carma_perception_msgs::msg::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
  } else if (
    in_msg.core_data.accuracy.presence_vector |
    carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) {
    // Position accuracy available

    heading_variance = 1.0;  // Yaw variance is not available so mark as
                             // impossible perfect case

    // Same calculation as shown in above condition. See that for description
    out_msg.confidence = 1.0 - std::min(1.0, fabs(largest_position_std / MAX_POSITION_STD));
    out_msg.presence_vector |=
      carma_perception_msgs::msg::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
  } else if (
    in_msg.core_data.accuracy.presence_vector |
    carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE) {
    // Orientation accuracy available

    lat_variance = 1.0;
    lon_variance = 1.0;
  }
  // Else: No accuracies available

  /////// Pose and Covariance /////////
  // Compute the pose

  if (
    (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE) &&
    (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE) &&
    (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE) &&
    (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::HEADING_AVAILABLE)) {
    double longitude = std::min(in_msg.core_data.longitude, in_msg.core_data.LONGITUDE_MAX);
    longitude = std::max(longitude, in_msg.core_data.LONGITUDE_MIN);

    double latitude = std::min(in_msg.core_data.latitude, in_msg.core_data.LATITUDE_MAX);
    // latitude =  std::max(latitude, in_msg.core_data.LATITUDE_MIN);

    double elev = std::min(in_msg.core_data.elev, in_msg.core_data.ELEVATION_MAX);
    // elev = std::max(elev, (float)in_msg.core_data.ELEVATION_MIN);

    double heading = std::min(in_msg.core_data.heading, in_msg.core_data.HEADING_MAX);
    // heading = std::max(heading, in_msg.core_data.HEADING_MIN);

    out_msg.pose = impl::pose_from_gnss(
      map_projector, ned_in_map_rotation, {latitude, longitude, elev}, heading, lat_variance,
      lon_variance, heading_variance);
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;
  }
  // Else: No pose available

  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;

  /////// Predictions /////////
  // Compute predictions
  // For prediction, if the prediction is available we will sample it
  // If not then assume linear motion

  std::vector<geometry_msgs::msg::Pose> predicted_poses;

  predicted_poses = impl::sample_2d_linear_motion(
    out_msg.pose.pose, out_msg.velocity.twist.linear.x, pred_period, pred_step_size);

  out_msg.predictions = impl::predicted_poses_to_predicted_state(
    predicted_poses, out_msg.velocity.twist.linear.x, rclcpp::Time(out_msg.header.stamp),
    rclcpp::Duration(pred_step_size * 1e9), map_frame_id, out_msg.confidence, out_msg.confidence);
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::PREDICTION_PRESENCE_VECTOR;
}

}  // namespace conversion
}  // namespace motion_computation
