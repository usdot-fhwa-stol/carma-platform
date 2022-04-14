#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <motion_computation/impl/bsm_to_external_object_helpers.hpp>
#include <motion_computation/message_conversions.hpp>

namespace motion_computation {

namespace conversion {

void convert(const carma_v2x_msgs::msg::BSM& in_msg, carma_perception_msgs::msg::ExternalObject& out_msg,
             const std::string& map_frame_id, double pred_period, double pred_step_size,
             const lanelet::projection::LocalFrameProjector& map_projector, tf2::Quaternion ned_in_map_rotation) {

  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;

  if (true) //TODO:: check how to determine an object is dynamic???
  {
    /////// Dynamic Object /////////
    out_msg.dynamic_obj = true;
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::DYNAMIC_OBJ_PRESENCE;

  }

  /////// Object Id /////////
  // Generate a unique object id from the psm id
  out_msg.id = 0;
  for (int i = in_msg.core_data.id.size() - 1; i >= 0; i--) {  // using signed iterator to handle empty case
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
  
  if (in_msg.core_data.size.presence_vector & carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE)
  {
    out_msg.size.x = in_msg.core_data.size.vehicle_length;
  }
  else out_msg.size.x = 2.5; //value from mob path to external obj conversion

  if (in_msg.core_data.size.presence_vector & carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE)
  {
    out_msg.size.y = in_msg.core_data.size.vehicle_width;
  }
  else out_msg.size.x = 2.25; //value from mob path to external obj conversion
  
  out_msg.size.z = 2.0; //value from mob path to external obj conversion

  out_msg.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;

  /////// Velocity /////////
  // Set the velocity
  if (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE)
  {
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::VELOCITY_PRESENCE_VECTOR;
    float_t speed_mps = in_msg.core_data.speed * 0.02;

    out_msg.velocity.twist.linear.x = std::min(speed_mps, in_msg.core_data.SPEED_MAX);
  }
  
    // NOTE: The velocity covariance is not provided in the PSM. In order to
  // compute it you need at least two BSM messages
  //     Tracking and associating BSM messages would be an increase in
  //     complexity for this conversion which is not warranted without an
  //     existing use case for the velocity covariance. If a use case is
  //     presented for it, such an addition can be made at that time.

  /////// Confidences /////////
  // Compute the position covariance
  // For computing confidence we will use the largest provided standard
  // deviation of position
  double largest_position_std = std::max(in_msg.core_data.accuracy.semi_major, in_msg.core_data.accuracy.semi_minor);

  double lat_variance = in_msg.core_data.accuracy.semi_minor * in_msg.core_data.accuracy.semi_minor;

  double lon_variance = in_msg.core_data.accuracy.semi_major * in_msg.core_data.accuracy.semi_major;

  double heading_variance = in_msg.core_data.accuracy.orientation * in_msg.core_data.accuracy.orientation;

  double position_confidence = 0.1;  // Default will be 10% confidence. If the position accuracy is
                                     // available then this value will be updated

  // A standard deviation which is larger than the acceptable value to give
  // 95% confidence interval on fitting the pedestrian within one 3.7m lane
  constexpr double MAX_POSITION_STD = 1.85;

  if ((in_msg.core_data.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) &&
      (in_msg.core_data.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE))
  {
    // Both accuracies available

    // NOTE: ExternalObject.msg does not clearly define what is meant by
    // position confidence
    //     Here we are providing a linear scale based on the positional accuracy
    //     where 0 confidence would denote A standard deviation which is larger
    //     than the acceptable value to give 95% confidence interval on fitting
    //     the pedestrian within one 3.7m lane
    // Set the confidence
    // Without a way of getting the velocity confidence from the PSM we will use
    // the position confidence for both
    out_msg.confidence = 1.0 - std::min(1.0, fabs(largest_position_std / MAX_POSITION_STD));
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
  } 
  else if (in_msg.core_data.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE)
  {
    // Position accuracy available

    heading_variance = 1.0;  // Yaw variance is not available so mark as
                             // impossible perfect case

    // Same calculation as shown in above condition. See that for description
    out_msg.confidence = 1.0 - std::min(1.0, fabs(largest_position_std / MAX_POSITION_STD));
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
  } 
  else if (in_msg.core_data.accuracy.presence_vector |
          carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE) 
  {
    // Orientation accuracy available

    lat_variance = 1.0;
    lon_variance = 1.0;
  }
  // Else: No accuracies available

  /////// Pose and Covariance /////////
  // Compute the pose

  if ((in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE) && 
      (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE) && 
      (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE) && 
      (in_msg.core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::HEADING_AVAILABLE))
  {
    out_msg.pose = impl::pose_from_gnss(map_projector, ned_in_map_rotation,
                                      {in_msg.core_data.latitude, in_msg.core_data.longitude, in_msg.core_data.elev},
                                      in_msg.core_data.heading, lat_variance, lon_variance, heading_variance);
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;
  }
  // Else: No pose available

  
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;

  /////// Predictions /////////
  // Compute predictions
  // For prediction, if the prediction is available we will sample it
  // If not then assume linear motion

  std::vector<geometry_msgs::msg::Pose> predicted_poses;

  predicted_poses = impl::sample_2d_linear_motion(out_msg.pose.pose, out_msg.velocity.twist.linear.x, pred_period, pred_step_size);

  out_msg.predictions = impl::predicted_poses_to_predicted_state(predicted_poses, out_msg.velocity.twist.linear.x, rclcpp::Time(out_msg.header.stamp),
                                                                 rclcpp::Duration(pred_step_size * 1e9), map_frame_id, out_msg.confidence,
                                                                 out_msg.confidence);
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::PREDICTION_PRESENCE_VECTOR;
}

namespace impl {

std::vector<geometry_msgs::msg::Pose> sample_2d_linear_motion(const geometry_msgs::msg::Pose& pose, double velocity,
                                                              double period, double step_size) 
{
  std::vector<geometry_msgs::msg::Pose> output;
  output.reserve((period / step_size) + 1);

  tf2::Vector3 pose_in_map_translation(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion pose_in_map_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Transform pose_in_map(pose_in_map_quat, pose_in_map_translation);

  double total_dt = 0;

  while (total_dt < period) {
    // Compute the 2d position and orientation in the Pose frame
    total_dt += step_size;
    double dx_from_start = velocity * total_dt;  // Assuming linear motion in pose frame

    double x = pose.position.x + dx_from_start;

    tf2::Vector3 position(x, 0, 0);

    // Convert the position and orientation in the pose frame to the map frame
    tf2::Transform pose_to_sample(tf2::Quaternion::getIdentity(), position);
    tf2::Transform map_to_sample = pose_in_map * pose_to_sample;

    geometry_msgs::msg::Pose sample_pose;

    sample_pose.position.x = map_to_sample.getOrigin().x();
    sample_pose.position.y = map_to_sample.getOrigin().y();
    sample_pose.position.z = map_to_sample.getOrigin().z();

    sample_pose.orientation.x = map_to_sample.getRotation().x();
    sample_pose.orientation.y = map_to_sample.getRotation().y();
    sample_pose.orientation.z = map_to_sample.getRotation().z();
    sample_pose.orientation.w = map_to_sample.getRotation().w();

    output.emplace_back(sample_pose);
  }

  return output;
}

// NOTE heading will need to be set after calling this
geometry_msgs::msg::PoseWithCovariance pose_from_gnss(const lanelet::projection::LocalFrameProjector& projector,
                                                 const tf2::Quaternion& ned_in_map_rotation, const lanelet::GPSPoint& gps_point,
                                                 const double& heading, const double lat_variance,
                                                 const double lon_variance, const double heading_variance) {
  //// Convert the position information into the map frame using the proj
  /// library
  lanelet::BasicPoint3d map_point = projector.forward(gps_point);

  geometry_msgs::msg::PoseWithCovariance pose;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "map_point: " << map_point.x() << ", " << map_point.y() << ", " << map_point.z());

  if (fabs(map_point.x()) > 10000.0 ||
      fabs(map_point.y()) > 10000.0) {  // Above 10km from map origin earth curvature will start to have a negative
                                        // impact on system performance

    RCLCPP_WARN_STREAM(rclcpp::get_logger("motion_computation::conversion"),
        "Distance from map origin is larger than supported by system "
        "assumptions. Strongly advise "
        "alternative map origin be used. ");
  }

  //// Convert the orientation information into the map frame
  // This logic assumes that the orientation difference between an NED frame
  // located at the map origin and an NED frame located at the GNSS point are
  // sufficiently small that they can be ignored. Therefore it is assumed the
  // heading report of the GNSS system regardless of its postion in the map
  // without change in its orientation will give the same result (as far as we
  // are concered).

  tf2::Quaternion R_m_n(ned_in_map_rotation);  // Rotation of NED frame in map
                                               // frame
  tf2::Quaternion R_n_h;                       // Rotation of sensor heading report in NED frame
  R_n_h.setRPY(0, 0, heading * wgs84_utils::DEG2RAD);

  tf2::Quaternion R_m_s = R_m_n * R_n_h;  // Rotation of sensor in map frame under assumption that
                                          // distance from map origin is sufficiently small so as to
                                          // ignore local changes in NED orientation

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "R_m_n (x,y,z,w) : ( " << R_m_n.x() << ", " << R_m_n.y() << ", " << R_m_n.z() << ", " << R_m_n.w());

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "R_n_h (x,y,z,w) : ( " << R_n_h.x() << ", " << R_n_h.y() << ", " << R_n_h.z() << ", " << R_n_h.w());

  // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "R_h_s (x,y,z,w) : ( " << R_h_s.x() << ", " << R_h_s.y() << ", " << R_h_s.z() << ", " << R_h_s.w());

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "R_m_s (x,y,z,w) : ( " << R_m_s.x() << ", " << R_m_s.y() << ", " << R_m_s.z() << ", " << R_m_s.w());

  tf2::Transform T_m_s(R_m_s, tf2::Vector3(map_point.x(), map_point.y(),
                                           map_point.z()));  // Reported position and orientation
                                                             // of sensor frame in map frame

        tf2::Transform T_m_n_no_heading(R_m_n, {0, 0, 0}); // Used to transform the covariance

        // This covariance represents the covariance of the NED frame N-lat,
        // E-lon, heading- angle east of north This means that the covariance is
        // in the NED frame, and needs to be transformed to the map frame
        // clang-format off
        std::array<double, 36> input_covariance = { 
          lat_variance, 0, 0,  0, 0, 0,
          0, lon_variance, 0,  0, 0, 0,
          0, 0, 0,  0, 0, 0,
          0,  0,  0,  1,  0, 0,
          0,  0,  0,  0,  1, 0, 
          0,  0,  0,  0,  0, heading_variance
        };
        // clang-format on

        std::array<double, 36> new_cov = tf2::transformCovariance(input_covariance,
                                     // Per the usage of transformCovariance in tf2_geometry_msgs
                                     // this frame should be the transform between the map which the pose
                                     // is in an the target frame.
                                     T_m_n_no_heading);

        // Populate message
        pose.pose.position.x = T_m_s.getOrigin().getX();
        pose.pose.position.y = T_m_s.getOrigin().getY();
        pose.pose.position.z = T_m_s.getOrigin().getZ();

        pose.pose.orientation.x = T_m_s.getRotation().getX();
        pose.pose.orientation.y = T_m_s.getRotation().getY();
        pose.pose.orientation.z = T_m_s.getRotation().getZ();
        pose.pose.orientation.w = T_m_s.getRotation().getW();

        pose.covariance = new_cov;

        return pose;
}

std::vector<carma_perception_msgs::msg::PredictedState> predicted_poses_to_predicted_state(
    const std::vector<geometry_msgs::msg::Pose>& poses, double constant_velocity, const rclcpp::Time& start_time,
    const rclcpp::Duration& step_size, const std::string& frame, double initial_pose_confidence,
    double initial_vel_confidence) {
  std::vector<carma_perception_msgs::msg::PredictedState> output;
  output.reserve(poses.size());

  rclcpp::Time time(start_time);

  for (auto p : poses) {
    time += step_size;
    carma_perception_msgs::msg::PredictedState pred_state;
    pred_state.header.stamp = time;
    pred_state.header.frame_id = frame;

    pred_state.predicted_position = p;
    pred_state.predicted_position_confidence = 0.9 * initial_pose_confidence;  // Reduce confidence by 10 % per timestep
    initial_pose_confidence = pred_state.predicted_position_confidence;

    pred_state.predicted_velocity.linear.x = constant_velocity;
    pred_state.predicted_velocity_confidence = 0.9 * initial_vel_confidence;  // Reduce confidence by 10 % per
                                                                                     // timestep
    initial_vel_confidence = pred_state.predicted_velocity_confidence;

    output.push_back(pred_state);
  }
  return output;
}

}  // namespace impl

}  // namespace conversion
}  // namespace motion_computation