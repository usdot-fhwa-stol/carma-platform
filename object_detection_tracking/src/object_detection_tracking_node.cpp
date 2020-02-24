/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include "object_detection_tracking_node.h"

namespace object{

  using std::placeholders::_1;

  ObjectDetectionTrackingNode::ObjectDetectionTrackingNode(): object_worker_(std::bind(&ObjectDetectionTrackingNode::publishObject, this, _1){}

  void ObjectDetectionTrackingNode::initialize()
  {
    autoware_obj_sub_=nh_.subscribe("detected_objects",10,&ObjectDetectionTrackingWorker::detectedObjectCallback,&object_worker_);
    carma_obj_pub_=nh_.advertise<cav_msgs::ExternalObjectList>("external_objects", 10);
  }

  void ObjectDetectionTrackingNode::publishObject(const cav_msgs::ExternalObjectList& obj_msg)
  {
  carma_obj_pub_.publish(obj_msg);
  }

  void ObjectDetectionTrackingNode::run()
  {
    initialize();
    nh_.setSpinRate(20);
    nh_.spin();
  }

}//object
