
#include <carma_transform_server/TransformServer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

TransformServer::TransformServer(int argc, char **argv) : tfListener_(tfBuffer_){

}

bool TransformServer::get_transform_cb(cav_srvs::GetTransform::Request  &req, cav_srvs::GetTransform::Response &res) {
  try{
    res.transform = tfBuffer_.lookupTransform(req.parent_frame, req.child_frame, ros::Time(0));
    res.errorStatus = res.NO_ERROR;
  }
  catch (tf2::TransformException &ex) {
    res.errorStatus = res.NO_TRANSFORM_EXISTS;
    ROS_WARN("TRANSFORM | Invalid transform request made to transform_server: %s", ex.what());
  }

return true;
}

int TransformServer::run() {
  // Setup get_transform service server
  get_transform_service_ = node_.advertiseService("get_transform", &TransformServer::get_transform_cb, this);

  // Spin
  ros::spin();
  return 0;
}