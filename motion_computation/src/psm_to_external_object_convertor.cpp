
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <motion_computation/impl/psm_to_external_object_helpers.hpp>
#include <motion_computation/message_conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <wgs84_utils/wgs84_utils.h>

namespace motion_computation {

namespace conversion {

void convert(const carma_v2x_msgs::msg::PSM& in_msg, carma_perception_msgs::msg::ExternalObject& out_msg,
             const std::string& map_frame_id, double pred_period, double pred_step_size,

             const lanelet::projection::LocalFrameProjector& map_projector, const tf2::Quaternion& ned_in_map_rotation, 
             rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock) {

  /////// Dynamic Object /////////
  out_msg.dynamic_obj = true;  // If a PSM is sent then the object is dynamic
                               // since its a living thing
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::DYNAMIC_OBJ_PRESENCE;

  /////// Object Id /////////
  // Generate a unique object id from the psm id
  out_msg.id = 0;
  for (int i = in_msg.id.id.size() - 1; i >= 0; i--) {  // using signed iterator to handle empty case
    // each byte of the psm id gets placed in one byte of the object id.
    // This should result in very large numbers which will be unlikely to
    // conflict with standard detections
    out_msg.id |= in_msg.id.id[i] << (8 * i);
  }
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::ID_PRESENCE_VECTOR;

  /////// BSM Id /////////
  // Additionally, store the id in the bsm_id field
  out_msg.bsm_id = in_msg.id.id;
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;

  /////// Timestamp /////////
  // Compute the timestamp

  auto psm_timestamp = impl::get_psm_timestamp(in_msg, node_clock->get_clock());
  out_msg.header.stamp = builtin_interfaces::msg::Time(psm_timestamp);
  out_msg.header.frame_id = map_frame_id;

  /////// Object Type and Size /////////
  // Set the type

  if (in_msg.basic_type.type == j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN ||
      in_msg.basic_type.type == j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PUBLIC_SAFETY_WORKER ||
      in_msg.basic_type.type == j2735_v2x_msgs::msg::PersonalDeviceUserType::AN_ANIMAL)  // Treat animals
                                                                                    // like people
                                                                                    // since we have
                                                                                    // no internal
                                                                                    // class for that
  {
    out_msg.object_type = carma_perception_msgs::msg::ExternalObject::PEDESTRIAN;

    // Default pedestrian size
    // Assume a
    // ExternalObject dimensions are half actual size
    // Here we assume 1.0, 1.0, 2.0
    out_msg.size.x = 0.5;
    out_msg.size.y = 0.5;
    out_msg.size.z = 1.0;

  } else if (in_msg.basic_type.type == j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDALCYCLIST) {
    out_msg.object_type =
        carma_perception_msgs::msg::ExternalObject::MOTORCYCLE;  // Currently external object cannot represent bicycles,
                                                                 // but motor cycle seems like the next best choice

    // Default bicycle size
    out_msg.size.x = 1.0;
    out_msg.size.y = 0.5;
    out_msg.size.z = 1.0;
  } else {
    out_msg.object_type = carma_perception_msgs::msg::ExternalObject::UNKNOWN;

    // Default pedestrian size
    out_msg.size.x = 0.5;
    out_msg.size.y = 0.5;
    out_msg.size.z = 1.0;
  }
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::SIZE_PRESENCE_VECTOR;

  /////// Velocity /////////
  // Set the velocity

  out_msg.velocity.twist.linear.x = in_msg.speed.velocity;
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::VELOCITY_PRESENCE_VECTOR;
  // NOTE: The velocity covariance is not provided in the PSM. In order to
  // compute it you need at least two PSM messages
  //     Tracking and associating PSM messages would be an increase in
  //     complexity for this conversion which is not warranted without an
  //     existing use case for the velocity covariance. If a use case is
  //     presented for it, such an addition can be made at that time.

  ///// Confidences /////////

  double largest_position_std = 0.0;
  double lat_variance = 0.0;
  double lon_variance = 0.0;
  double heading_variance = 0.0;  // Yaw variance is not available so mark as
                             // impossible perfect case
  double position_confidence = 0.0;  // Default will be 10% confidence. If the position accuracy is
                                      // available then this value will be updated

    // A standard deviation which is larger than the acceptable value to give
  // 95% confidence interval on fitting the pedestrian within one 3.7m lane
  constexpr double MAX_POSITION_STD = 1.85;
  
  if ((in_msg.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) &&
      (in_msg.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE)) {
    // Both accuracies available

    // Compute the position covariance
    // For computing confidence we will use the largest provided standard
    // deviation of position
    largest_position_std = std::max(in_msg.accuracy.semi_major, in_msg.accuracy.semi_minor);
    lat_variance = in_msg.accuracy.semi_minor * in_msg.accuracy.semi_minor;
    lon_variance = in_msg.accuracy.semi_major * in_msg.accuracy.semi_major;
    heading_variance = in_msg.accuracy.orientation * in_msg.accuracy.orientation;

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
  } else if (in_msg.accuracy.presence_vector | carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) {
    // Position accuracy available


    largest_position_std = std::max(in_msg.accuracy.semi_major, in_msg.accuracy.semi_minor);
    lat_variance = in_msg.accuracy.semi_minor * in_msg.accuracy.semi_minor;
    lon_variance = in_msg.accuracy.semi_major * in_msg.accuracy.semi_major;
    heading_variance = 1.0;  // Yaw variance is not available so mark as
                             // impossible perfect case

    // Same calculation as shown in above condition. See that for description
    out_msg.confidence = 1.0 - std::min(1.0, fabs(largest_position_std / MAX_POSITION_STD));
    out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
    
  } else if (in_msg.accuracy.presence_vector |
             carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE) {
    // Orientation accuracy available

    lat_variance = 1.0;
    lon_variance = 1.0;

    heading_variance = in_msg.accuracy.orientation * in_msg.accuracy.orientation;
  }
  else{ //No accuracies available
    lat_variance = 1.0;
    lon_variance = 1.0;
    heading_variance = 1.0;
  }

  /////// Pose and Covariance /////////
  // Compute the pose
  lanelet::GPSPoint gps_point({in_msg.position.latitude, in_msg.position.longitude, in_msg.position.elevation});
  out_msg.pose = impl::pose_from_gnss(map_projector, ned_in_map_rotation, gps_point,
                                      in_msg.heading.heading, lat_variance, lon_variance, heading_variance);
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;

  /////// Predictions /////////
  // Compute predictions
  // For prediction, if the prediction is available we will sample it
  // If not then assume linear motion

  std::vector<geometry_msgs::msg::Pose> predicted_poses;

  if (in_msg.presence_vector & carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION) {
    // Based on the vehicle frame used in j2735 positive should be to the right
    // and negative to the left
    predicted_poses =
        impl::sample_2d_path_from_radius(out_msg.pose.pose, out_msg.velocity.twist.linear.x,
                                         -in_msg.path_prediction.radius_of_curvature, pred_period, pred_step_size);
  } else {
    predicted_poses =
        impl::sample_2d_linear_motion(out_msg.pose.pose, out_msg.velocity.twist.linear.x, pred_period, pred_step_size);
  }

  out_msg.predictions = impl::predicted_poses_to_predicted_state(predicted_poses, out_msg.velocity.twist.linear.x, rclcpp::Time(out_msg.header.stamp),
                                                                 rclcpp::Duration(pred_step_size * 1e9), map_frame_id,  out_msg.confidence,
                                                                 out_msg.confidence);
  out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::PREDICTION_PRESENCE_VECTOR;
}

namespace impl {
std::vector<geometry_msgs::msg::Pose> sample_2d_path_from_radius(const geometry_msgs::msg::Pose& pose, double velocity,
                                                                 double radius_of_curvature, double period,
                                                                 double step_size) {
  std::vector<geometry_msgs::msg::Pose> output;
  output.reserve((period / step_size) + 1);

  tf2::Vector3 pose_in_map_translation(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion pose_in_map_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Transform pose_in_map(pose_in_map_quat, pose_in_map_translation);

  // The radius of curvature originates from the frame of the provided pose
  // So the turning center is at (0, r)
  double center_x_in_pose = 0;
  double center_y_in_pose = radius_of_curvature;

  double total_dt = 0;

  while (total_dt < period) {
    // Compute the 2d position and orientation in the Pose frame
    total_dt += step_size;
    double delta_arc_length = velocity * total_dt;  // Assumes perfect point motion along curve

    double turning_angle = delta_arc_length / radius_of_curvature;
    double dx_from_center = radius_of_curvature * sin(turning_angle);
    double dy_from_center = radius_of_curvature * cos(turning_angle);

    double x = center_x_in_pose + dx_from_center;
    double y = center_y_in_pose + dy_from_center;

    tf2::Vector3 position(x, y, 0);

    tf2::Quaternion quat;
    quat.setRPY(0, 0, turning_angle);

    // Convert the position and orientation in the pose frame to the map frame
    tf2::Transform pose_to_sample(quat, position);
    tf2::Transform map_to_sample = pose_in_map * pose_to_sample;

    geometry_msgs::msg::Pose sample_pose;

    sample_pose.position.x = map_to_sample.getOrigin().x();
    sample_pose.position.y = map_to_sample.getOrigin().y();
    sample_pose.position.z = pose.orientation.z;  // Reuse the z position from the initial pose

    sample_pose.orientation.x = map_to_sample.getRotation().x();
    sample_pose.orientation.y = map_to_sample.getRotation().y();
    sample_pose.orientation.z = map_to_sample.getRotation().z();
    sample_pose.orientation.w = map_to_sample.getRotation().w();

    output.emplace_back(sample_pose);
  }

  return output;
}

std::vector<geometry_msgs::msg::Pose> sample_2d_linear_motion(const geometry_msgs::msg::Pose& pose, double velocity,
                                                              double period, double step_size) {
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

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"), "R_m_s (x,y,z,w) : ( " << R_m_s.x() << ", " << R_m_s.y() << ", " << R_m_s.z() << ", " << R_m_s.w());

  tf2::Transform T_m_s(R_m_s, tf2::Vector3(map_point.x(), map_point.y(),
                                           map_point.z()));  // Reported position and orientation
                                                             // of sensor frame in map frame


        tf2::Transform T_m_n_no_heading(R_m_n, tf2::Vector3(0, 0, 0)); // Used to transform the covariance

        // This covariance represents the covariance of the NED frame N-lat,
        // E-lon, heading- angle east of north This means that the covariance is
        // in the NED frame, and needs to be transformed to the map frame
        // clang-format off
        std::array<double, 36> input_covariance = { 
          lat_variance, 0, 0,  0, 0, 0,
          0, lon_variance, 0,  0, 0, 0,
          0, 0, 1,  0, 0, 0,
          0,  0,  0,  1,  0, 0,
          0,  0,  0,  0,  1, 0, 
          0,  0,  0,  0,  0, heading_variance
        };
        // clang-format on

        std::array<double, 36> new_cov =
            tf2::transformCovariance(input_covariance,
                                     // Per the usage of transformCovariance in tf2_geometry_msgs
                                     // this frame should be the transform between the map which the pose
                                     // is in an the target frame.
                                     T_m_n_no_heading);

        // Populate message
        geometry_msgs::msg::PoseWithCovariance pose;
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

rclcpp::Time get_psm_timestamp(const carma_v2x_msgs::msg::PSM& in_msg, rclcpp::Clock::SharedPtr clock) {
    boost::posix_time::ptime utc_time_of_current_psm;

  // Get the utc epoch start time
  static const boost::posix_time::ptime inception_boost(boost::posix_time::time_from_string("1970-01-01 00:00:00.000"));

  // Determine if the utc time of the path history can be used instead of the
  // sec_mark The sec_mark is susceptible to large error on minute transitions
  // due to missing "minute of the year" field If the second mark in the path
  // history is identical and the full utc time is provided with ms resolution
  // then it can be assumed the initial_position is the same as the PSM data and
  // the utc_time can be used instead of sec_mark
  // clang-format off
  if ((in_msg.presence_vector & 
        carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY) &&
        (in_msg.path_history.presence_vector & 
        carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::YEAR) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MONTH) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::DAY) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::HOUR) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MINUTE) &&
        (in_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::SECOND) &&
      (in_msg.sec_mark.millisecond == in_msg.path_history.initial_position.utc_time.second.millisecond)) {
    // clang-format on
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("motion_computation::conversion"),
                        "Using UTC time of path history to determine PSM "
                        "timestamp. Assumed valid since UTC is fully specified "
                        "and sec_mark == utc_time.seconds in this message.");

    boost::posix_time::time_duration time_of_day = boost::posix_time::hours(in_msg.path_history.initial_position.utc_time.hour.hour) +
                                                   boost::posix_time::minutes(in_msg.path_history.initial_position.utc_time.minute.minute) +
                                                   boost::posix_time::milliseconds(in_msg.path_history.initial_position.utc_time.second.millisecond);

    boost::gregorian::date utc_day(in_msg.path_history.initial_position.utc_time.year.year,
                                   in_msg.path_history.initial_position.utc_time.month.month,
                                   in_msg.path_history.initial_position.utc_time.day.day);

    utc_time_of_current_psm = boost::posix_time::ptime(utc_day) + time_of_day;
  } else {  // If the utc time of the path history cannot be used to account for minute
            // change over, then we have to default to the sec mark

    RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("motion_computation::conversion"), *clock.get(), rclcpp::Duration(5, 0).nanoseconds(),
                                "PSM PathHistory utc timstamp does not match "
                                "sec_mark. Unable to determine the minute of "
                                "the year used for PSM data. Assuming local "
                                "clock is exactly synced. This is NOT "
                                "ADVISED.");

    // Get the current ROS time
    auto current_time = clock->now();

    // Convert the ros time to a boost duration
    boost::posix_time::time_duration duration_since_inception(lanelet::time::durationFromSec(current_time.seconds()));

    // Get the current ROS time in UTC
    auto curr_time_boost = inception_boost + duration_since_inception;

    // Get duration of current day
    auto duration_in_day_till_current_time = curr_time_boost.time_of_day();

    // Extract hours and minutes
    long hour_count_in_day = duration_in_day_till_current_time.hours() ;
    long minute_count_in_hour = duration_in_day_till_current_time.minutes();

    // Get the duration of the minute in the day
    auto start_of_minute_in_day = boost::posix_time::hours(hour_count_in_day) + boost::posix_time::minutes(minute_count_in_hour);

    // Get the start of the day in ROS time
    boost::posix_time::ptime start_of_day(curr_time_boost.date());

    // Ge the start of the current minute in ROS time
    boost::posix_time::ptime utc_start_of_current_minute = start_of_day + start_of_minute_in_day;

    // Compute the UTC PSM stamp from the sec_mark using ROS time as the clock

    boost::posix_time::time_duration s_in_cur_minute = boost::posix_time::milliseconds(in_msg.sec_mark.millisecond);

    utc_time_of_current_psm = utc_start_of_current_minute + s_in_cur_minute;
  }

  boost::posix_time::time_duration nsec_since_epoch = utc_time_of_current_psm - inception_boost;

  if (nsec_since_epoch.is_special()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("motion_computation::conversion"),
                        "Computed psm nsec_since_epoch is special (computation "
                        "failed). Value effectively undefined.");
  }

  return rclcpp::Time(nsec_since_epoch.total_nanoseconds());

}
}  // namespace impl

}  // namespace conversion
}  // namespace motion_computation