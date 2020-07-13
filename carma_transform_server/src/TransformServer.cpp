/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <carma_transform_server/TransformServer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

TransformServer::TransformServer(int argc, char **argv) : tfListener_(tfBuffer_){

}

bool TransformServer::get_transform_cb(cav_srvs::GetTransform::Request  &req, cav_srvs::GetTransform::Response &res) {
  try{
    res.transform = tfBuffer_.lookupTransform(req.parent_frame, req.child_frame, req.stamp);
    res.error_status = res.NO_ERROR;
  } catch (tf2::ExtrapolationException& ex) {
    ROS_WARN("| Transform Server | TRANSFORM | transform_server could not extrapolate requested transform. Using latest transform available%s", ex.what());

    try {
      res.transform = tfBuffer_.lookupTransform(req.parent_frame, req.child_frame, ros::Time(0));
      res.error_status = res.COULD_NOT_EXTRAPOLATE;
    } catch (tf2::TransformException &ex) {
      res.error_status = res.NO_TRANSFORM_EXISTS;
      ROS_WARN("| Transform Server | TRANSFORM | Invalid transform request made to transform_server: %s", ex.what());
    }

  } catch (tf2::TransformException &ex) {
    res.error_status = res.NO_TRANSFORM_EXISTS;
    ROS_WARN("| Transform Server | TRANSFORM | Invalid transform request made to transform_server: %s", ex.what());
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