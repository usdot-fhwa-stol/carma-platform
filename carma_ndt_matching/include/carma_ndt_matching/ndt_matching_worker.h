#pragma once

#include <geometry_msgs/Pose.h>
#include <functional>
#include <memory>
#include <boost/circular_buffer.hpp>
#include <algorithm>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
#include <motion_predict/predict_ctrv.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <limits>
#include "ndt_matching_config.h"

struct KinematicState {
  ros::Time stamp;
  geometry_msgs::Pose pose;
  geometry_msgs::Twist twist;
};

struct NDTResult {
  bool has_converged = false;
  int iteration_count = 0;
  double fitness_score = 0.0;
  double transform_probability = 0.0;
  double align_time = 0.0; // ms
  geometry_msgs::Pose pose;
  ros::Time stamp;
  std::string frame_id;
};

using LookupTransform = std::function<geometry_msgs::TransformStamped(const std::string&, const std::string&, const ros::Time&)>;
using ResultPublisher = std::function<void(const NDTResult&)>;

class NDTMatchingWorker {
  bool base_map_set_ = false;
  pcl::PointCloud<pcl::PointXYZ> map_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_solver_;
  sensor_msgs::PointCloud2ConstPtr prev_scan_msg_ptr_;

  NDTConfig config_;

  LookupTransform lookup_transform_;
  ResultPublisher result_pub_;

  bool baselink_in_sensor_set_ = false;
  Eigen::Matrix4f baselink_in_sensor_;  

  boost::circular_buffer<KinematicState> state_buffer_{10}; // TODO make size configurable or computed based on parameter rate differences

  public:
  NDTMatchingWorker(NDTConfig config, LookupTransform lookup_tf_function, ResultPublisher pose_pub) : config_(config), lookup_transform_(lookup_tf_function), result_pub_(pose_pub) {

  }

  // Core Algorithm
  // 0. Get base map
  // 1. Get new scan
  // 2. Get prev state
  // 3. Estimate current state
  // 4. Run NDT to find actual state
  // 5. Repeat

  void baseMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_msg) {
    // Store map
    ROS_ERROR_STREAM("Processing Map Message");

    pcl::fromROSMsg(*map_msg, map_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map_));

    // Setting point cloud to be aligned to.
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    new_ndt.setResolution(config_.resolution);
    new_ndt.setInputTarget(map_ptr);
    new_ndt.setMaximumIterations(config_.max_iter);
    new_ndt.setStepSize(config_.step_size);
    new_ndt.setTransformationEpsilon(config_.trans_eps);

    new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

    ndt_solver_ = new_ndt;
    base_map_set_ = true;

    ROS_ERROR_STREAM("Done Processing Map Message");

  }

  Eigen::Matrix4f baselinkPoseToEigenSensor(const geometry_msgs::Pose& pose) {

    Eigen::Affine3d pose_as_affine;
    tf2::fromMsg(pose, pose_as_affine);

    Eigen::Matrix4f pose_as_mat = pose_as_affine.matrix().cast<float>();

    return pose_as_mat * baselink_in_sensor_.inverse();
  }

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& scan) {

    // Get the transform to the lidar scan
    if (!baselink_in_sensor_set_) {
      try {
        Eigen::Affine3d pose_as_affine;
        pose_as_affine = tf2::transformToEigen(lookup_transform_(scan->header.frame_id, config_.baselink_frame_id_, ros::Time(0)));
        baselink_in_sensor_ = pose_as_affine.matrix().cast<float>();
        baselink_in_sensor_set_ = true;
        ROS_ERROR_STREAM("GOT baselink_lidar transform");
      } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM("NDT Ignoring scan message: Could not locate static transforms with exception " << ex.what());
        return;
      }
    }

    // Store the pointcloud for later use
    prev_scan_msg_ptr_ = scan;
  }



  KinematicState predictCurrentState(KinematicState prev_state, ros::Time target_time) {

    ros::Duration delta_t = target_time - prev_state.stamp;

    // Use motion prediction library
    cav_msgs::PredictedState prev_state_msg;
    prev_state_msg.predicted_position = prev_state.pose;
    prev_state_msg.predicted_velocity = prev_state.twist;

    cav_msgs::PredictedState predicted_msg = motion_predict::ctrv::predictStep(prev_state_msg, delta_t.toSec(), 0.1);

    KinematicState result;
    result.stamp = target_time;
    result.pose = predicted_msg.predicted_position;
    result.twist = predicted_msg.predicted_velocity;

    return result;
  }

  NDTResult optimizePredictedState(const Eigen::Matrix4f& pose, pcl::PointCloud<pcl::PointXYZ>::Ptr scanPtr) {
    ROS_ERROR_STREAM("Optimizing");
    NDTResult result;

    ndt_solver_.setInputSource(scanPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f map_to_sensor(Eigen::Matrix4f::Identity());   // base_link// TODO rename variables

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end;

    align_start = std::chrono::system_clock::now();
    ndt_solver_.align(*output_cloud, pose);
    align_end = std::chrono::system_clock::now();

    result.has_converged = ndt_solver_.hasConverged();

    map_to_sensor = ndt_solver_.getFinalTransformation();
    result.iteration_count = ndt_solver_.getFinalNumIteration();

    result.fitness_score = ndt_solver_.getFitnessScore();

    result.transform_probability = ndt_solver_.getTransformationProbability();

    result.align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
  
    Eigen::Matrix4f map_to_baselink = map_to_sensor * baselink_in_sensor_;  // T_map_baselink = T_map_lidar * T_lidar_baselink

    Eigen::Affine3d tf_as_affine;
    tf_as_affine.matrix() = map_to_baselink.cast<double>();

    //result.pose = Eigen::toMsg(tf_as_affine)
    tf2::convert(tf_as_affine, result.pose); // Convert to pose type
    ROS_ERROR_STREAM("Done Optimizing");
    return result;
  }

  KinematicState findPrevState(const ros::Time& time) {
    ROS_ERROR_STREAM("Finding previous state for time: " << time.toSec());
    // Finds the lower bound in at most log(last - first) + 1 comparisons
    // Find the first state which is more recent than our time
    auto state_after_time = std::lower_bound(state_buffer_.begin(), state_buffer_.end(), time, 
      [] (const KinematicState& state, const ros::Time& target_time) -> bool {
        return state.stamp < target_time;
    });


    if (state_after_time == state_buffer_.end()) { // If the most recent state is still older than our time
      return state_buffer_.back(); // Return the most recent state

    } else if (state_after_time == state_buffer_.begin()) { // If the oldest state is more recent than our time
      return state_buffer_.front(); // Try to use the most recent state without prediction
    }

    auto state_before_time = state_after_time - 1;

    ROS_ERROR_STREAM("Found state with time: " << (*state_after_time).stamp.toSec());

    return *state_before_time;
    
  }

  bool computeAndPublishUpdate() {

    // TODO get 
        // TODO how to synchronize the point cloud.
    // I am thinking it might make sense to store a buffer of twist results. Then use the one nearest the point cloud as the prev state
    // After computing the result we apply a basic motion prediction to the output to get the newest prediction
    if (!prev_scan_msg_ptr_ || !base_map_set_ || state_buffer_.empty()) {
      ROS_ERROR_STREAM("Waiting for data ");
      return true;
    }

    ros::Time start = ros::Time::now();

    KinematicState prev_state = findPrevState(prev_scan_msg_ptr_->header.stamp);

    KinematicState predicted_state; 

    if (prev_state.stamp > prev_scan_msg_ptr_->header.stamp) {
      ROS_ERROR_STREAM("NDT Pose De-sync. Oldest pose in buffer is more recent than point cloud");
      predicted_state = prev_state;
    } else {
      predicted_state = predictCurrentState(prev_state, prev_scan_msg_ptr_->header.stamp);
    }

    // TODO remove block
    ROS_ERROR_STREAM("PrevState [ " << prev_state.pose.position.x << ", " << prev_state.pose.position.y << ", " << prev_state.pose.position.z << " ]");
    ROS_ERROR_STREAM("predicted_state [ " << predicted_state.pose.position.x << ", " << predicted_state.pose.position.y << ", " << predicted_state.pose.position.z << " ]");

    Eigen::Affine3d pose_as_affine;
    Eigen::Matrix4f pose_as_mat = baselinkPoseToEigenSensor(predicted_state.pose);
    pose_as_affine.matrix() = pose_as_mat.cast<double>();
    geometry_msgs::Pose tempPose;
    tf2::convert(pose_as_affine, tempPose);

    ROS_ERROR_STREAM("tempPose [ " << tempPose.position.x << ", " << tempPose.position.y << ", " << tempPose.position.z << " ]");

    // TODO end remove block

    ros::Time pred_end = ros::Time::now();
    ROS_ERROR_STREAM("Prediction Time: " << (pred_end - start).toSec());

    ros::Time point_conv_start = ros::Time::now();
    // Convert the scan to pcl data type and store pointer to it
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;
    pcl::fromROSMsg(*prev_scan_msg_ptr_, filtered_scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));

    ros::Time point_conv_end = ros::Time::now();
    ROS_ERROR_STREAM("Conversion Time: " << (point_conv_end - point_conv_start).toSec());

    ros::Time op_start = ros::Time::now();
    NDTResult optimized_state = optimizePredictedState(baselinkPoseToEigenSensor(predicted_state.pose), prev_scan_ptr);
    ros::Time op_end = ros::Time::now();
    ROS_ERROR_STREAM("Optimization Time: " << (op_end - op_start).toSec());
    optimized_state.stamp = prev_scan_msg_ptr_->header.stamp;
    optimized_state.frame_id = config_.map_frame_id_;

    result_pub_(optimized_state);

    ros::Time end = ros::Time::now();

    ROS_ERROR_STREAM("CB Time: " << (end - start).toSec());
    // TODO publish other metrics
    return true;
  }

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    if (!prev_scan_msg_ptr_ || !base_map_set_) {
      ROS_WARN("Initial Pose not processed due to missing point scan or map data");
      return;
    }

     geometry_msgs::PoseStamped initial_pose;
     initial_pose.header = msg->header;
     initial_pose.pose = msg->pose.pose;

    geometry_msgs::TwistStamped initial_twist;
    initial_twist.header = initial_pose.header;
    initial_twist.header.frame_id = config_.baselink_frame_id_;

    double min_distance = std::numeric_limits<double>::max();
    double nearest_z = initial_pose.pose.position.z;
    for (const auto& p : map_)
    {
      double distance = hypot(initial_pose.pose.position.x - p.x, initial_pose.pose.position.y - p.y);
      if (distance < min_distance)
      {
        min_distance = distance;
        nearest_z = p.z;
      }
    }
    initial_pose.pose.position.z = nearest_z;

    geometry_msgs::TwistStampedConstPtr twist_ptr(new geometry_msgs::TwistStamped(initial_twist));
    geometry_msgs::PoseStampedConstPtr pose_ptr(new geometry_msgs::PoseStamped(initial_pose));

    ROS_ERROR_STREAM("Found initial pose z of " << initial_pose.pose.position.z);
    state_buffer_.clear();
    prevStateCallback(pose_ptr, twist_ptr);

  }
   

  void prevStateCallback(const geometry_msgs::PoseStampedConstPtr& pose, const geometry_msgs::TwistStampedConstPtr& twist) {
    if (pose->header.stamp != twist->header.stamp) {
      ROS_ERROR_STREAM("Received pose and twist with differing timestamps");
      // TODO This should never occur should an exception be thrown?
      return;
    }

    KinematicState state;
    state.stamp = pose->header.stamp;
    state.pose = pose->pose;
    state.twist = twist->twist;

    if (!state_buffer_.empty() && state_buffer_.back().stamp > state.stamp) {
      ROS_ERROR_STREAM("Received pose and twist messages older than previous message.");
      // TODO throw exception or find way to insert
      return;
    }

    state_buffer_.push_back(state);
  }

};