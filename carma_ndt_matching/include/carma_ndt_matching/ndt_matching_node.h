#include <functional>
#include <memory>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <carma_utils/CARMAUtils.h>


#include "ndt_matching_worker.h"
#include "ndt_matching_config.h"


class NDTMatchingNode {
  std::unique_ptr<NDTMatchingWorker> worker_;
  ros::CARMANodeHandle nh_;
  ros::CARMANodeHandle pnh_("~");

  // Buffer which holds the tree of transforms
  tf2_ros::Buffer tfBuffer_;
  // tf2 listeners. Subscribes to the /tf and /tf_static topics
  tf2_ros::TransformListener tfListener_(tfBuffer_);

  typedef message_filters::sync_policies::ExactTime <geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> StateSyncPolicy;
  typedef message_filters::Synchronizer<StateSyncPolicy> StateSynchronizer;

  // Since the CARMANodeHandle cannot wrap the message filter's subscribers this function serves as an intermediate for exception handling
  void poseTwistCallback(const geometry_msgs::PoseStampedConstPtr& pose, const geometry_msgs::TwistStamped& twist) {
    try {
      worker->prevStateCallback(pose, twist);
    } catch (const std::exception& e) {
      ros::CARMANodeHandle::handleException(e);
    }
  }

  void run() {

    ros::Publisher ndt_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);

    NDTConfig worker_config;

    // Load algorithm params
    pnh_->param<double>("ndt_max_iterations", worker_config.max_iter, worker_config.max_iter);
    pnh_->param<double>("ndt_resolution", worker_config.resolution, worker_config.resolution);
    pnh_->param<double>("ndt_step_size", worker_config.step_size, worker_config.step_size);
    pnh_->param<double>("ndt_max_iterations", worker_config.trans_eps, worker_config.trans_eps);

    // Load architecture params
    pnh_->param<std::string>("baselink_frame_id_", worker_config.baselink_frame_id_, worker_config.baselink_frame_id_);
    pnh_->param<std::string>("map_frame_id_", worker_config.map_frame_id_, worker_config.map_frame_id_);

    worker_.reset(new NDTMatchingWorker(worker_config, 
        std::bind(&tf2_ros::Buffer::lookupTransform, &tfBuffer_, _1, _2),
        std::bind(&ros::Publisher::publish, &ndt_pose_pub, _1))
    );

    // EKF Pose Subscriber
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh_, "ekf_pose", 10); // TODO do these need to be 10 if the policy is also 10?
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(nh_, "ekf_twist", 10);

    StateSynchronizer state_synchronizer(StateSyncPolicy(10), pose_sub, twist_sub);

    sync.registerCallback(boost::bind(&NDTMatchingNode::poseTwistCallback, this, _1, _2));

    // Points subscriber 
    ros::Subscriber points_sub = nh_.subscribe("filtered_points", 1, &NDTMatchingWorker::scanCallback, worker_.get());
    ros::Subscriber map_sub = nh_.subscribe("points_map", 1, &NDTMatchingWorker::baseMapCallback, worker_.get());


    // Spin
    cnh_.setSpinRate(10);
    cnh_.setSpinCallback(boost::bind(&NDTMatchingWorker::computeAndPublishUpdate, worker_.get()));
    cnh_.spin();
  }

};