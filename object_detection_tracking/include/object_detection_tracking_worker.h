#pragma once

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

#ifndef EXTERNAL_OBJECT_WORKER_H
#define EXTERNAL_OBJECT_WORKER_H

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>  

namespace object{

class ObjectDetectionTrackingWorker
{
private:
    // local copy of external object publihsers
    ros::Publisher pub_object_;

    cav_msgs::DriverStatus status_;
    cav_msgs::TrailerAngle tangle_;
 

 public:
    /*! \fn detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg)
    \brief detectedObjectCallback populates detected object along with its velocity,yaw, yaw_rate and static/dynamic class to DetectedObject message.
    \param  msg array of detected objects.
    */
  void detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg);

    /*! \fn set_publishers
    \brief Method to pass publishers into object worker class
    \param pub_object external object array publisher
    */
  void set_publishers(ros::Publisher pub_object);
 
};

}

#endif /* EXTERNAL_OBJECT_WORKER_H */