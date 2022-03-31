#include <carma_v2x_msgs/psm.hpp>
#include <carma_perception_msgs/external_object.hpp>

namespace object
{

namespace conversion
{

namespace impl // Namespace for functionality not meant to be part of public API but valueable to unit test in isolation
{
  std::vector<geometry_msgs::msg::Pose> sample_2d_path_from_radius(
  const geometry_msgs::msg::Pose &pose, double velocity,
  double radius_of_curvature, double period, double step_size);

  std::vector<geometry_msgs::msg::Pose> sample_2d_linear_motion(
    const geometry_msgs::msg::Pose &pose, double velocity, double period, double step_size);

  geometry_msgs::PoseWithCovariance pose_from_gnss(
    const lanelet::projection::LocalFrameProjector& projector, 
    const tf2::Quaternion& ned_in_map_rotation,
    const GPSPoint& gps_point, 
    const double& heading);

  std::vector<carma_perception_msgs::PredictedState> predicted_poses_to_predicted_state(
    const std::vector<geometry_msgs::msg::Pose> &poses, double constant_velocity, const rclcpp::Time &start_time, const rclcpp::Duration &step_size, const std::string &frame,
    double initial_pose_confidence, double initial_vel_confidence);

  rclcpp::Time get_psm_timestamp(const carma_v2x_msgs::msg::PSM &in_msg);
    
}
}
}