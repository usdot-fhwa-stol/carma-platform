#pragma once
#include <functional>
#include <memory>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include "ndt_matching_worker.h"
#include "ndt_matching_config.h"

namespace ph = std::placeholders;

class NDTMatchingNode {

  public: 
  std::unique_ptr<NDTMatchingWorker> worker_;
  ros::CARMANodeHandle nh_;
  ros::CARMANodeHandle pnh_{"~"};

  // Buffer which holds the tree of transforms
  tf2_ros::Buffer tfBuffer_;
  // tf2 listeners. Subscribes to the /tf and /tf_static topics
  tf2_ros::TransformListener tfListener_{tfBuffer_};

  ros::Publisher ndt_pose_pub_;
  ros::Publisher matching_time_pub_;

  typedef message_filters::sync_policies::ExactTime <geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> StateSyncPolicy;
  typedef message_filters::Synchronizer<StateSyncPolicy> StateSynchronizer;

  // Since the CARMANodeHandle cannot wrap the message filter's subscribers this function serves as an intermediate for exception handling
  void poseTwistCallback(const geometry_msgs::PoseStampedConstPtr& pose, const geometry_msgs::TwistStampedConstPtr& twist) {
    try {
      worker_->prevStateCallback(pose, twist);
    } catch (const std::exception& e) {
      ros::CARMANodeHandle::handleException(e);
    }
  }

  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame,const std::string& source_frame,const ros::Time& time) {
    return tfBuffer_.lookupTransform(target_frame, source_frame, time);
  }

  void publishResults(const NDTResult& results) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = results.pose;
    pose_msg.header.stamp = results.stamp;
    pose_msg.header.frame_id = results.frame_id;
    ndt_pose_pub_.publish(pose_msg);

    std_msgs::Float32 msg;
    msg.data = results.align_time;
    matching_time_pub_.publish(msg);
  }
//using LookupTransform = std::function<geometry_msgs::TransformStamped(const std::string &target_frame, const std::string &source_frame, const ros::Time &time)>;

  void run() {

    ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    matching_time_pub_ = nh_.advertise<std_msgs::Float32>("carma_ndt_matching_time", 10);

    NDTConfig worker_config;

    // Load algorithm params
    pnh_.param<int>("ndt_max_iterations", worker_config.max_iter, worker_config.max_iter);
    pnh_.param<double>("ndt_resolution", worker_config.resolution, worker_config.resolution);
    pnh_.param<double>("ndt_step_size", worker_config.step_size, worker_config.step_size);
    pnh_.param<double>("ndt_trans_eps", worker_config.trans_eps, worker_config.trans_eps);

    // Load architecture params
    pnh_.param<std::string>("baselink_frame_id_", worker_config.baselink_frame_id_, worker_config.baselink_frame_id_);
    pnh_.param<std::string>("map_frame_id_", worker_config.map_frame_id_, worker_config.map_frame_id_);

    worker_.reset(new NDTMatchingWorker(worker_config, 
        std::bind(&NDTMatchingNode::lookupTransform, this, ph::_1,ph::_2, ph::_3),
        std::bind(&NDTMatchingNode::publishResults, this, ph::_1))
    );
//std::bind(static_cast<void(ros::Publisher::*)(const geometry_msgs::PoseStamped&)(&ros::Publisher::publish), &ndt_pose_pub, ph::_1))
//        std::bind(static_cast<geometry_msgs::TransformStamped(tf2_ros::Buffer::*)(const std::string&,const std::string&,const ros::Time&)>(&tf2_ros::Buffer::lookupTransform), &tfBuffer_, ph::_1,ph::_2, ph::_3),

    // EKF Pose Subscriber
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh_, "ekf_pose", 10); // TODO do these need to be 10 if the policy is also 10?
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(nh_, "ekf_twist", 10);

    StateSynchronizer state_synchronizer(StateSyncPolicy(10), pose_sub, twist_sub);

    state_synchronizer.registerCallback(boost::bind(&NDTMatchingNode::poseTwistCallback, this, _1, _2));

    // Points subscriber 
    ros::Subscriber points_sub = nh_.subscribe("filtered_points", 1, &NDTMatchingWorker::scanCallback, worker_.get());
    ros::Subscriber map_sub = nh_.subscribe("points_map", 1, &NDTMatchingWorker::baseMapCallback, worker_.get());


    // Spin
    nh_.setSpinRate(10);
    nh_.setSpinCallback(boost::bind(&NDTMatchingWorker::computeAndPublishUpdate, worker_.get()));
    nh_.spin();
  }

};