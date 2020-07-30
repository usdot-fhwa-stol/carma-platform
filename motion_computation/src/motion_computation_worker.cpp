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
#include "motion_computation_worker.h"
#include <motion_predict/motion_predict.h>
#include <motion_predict/predict_ctrv.h>

namespace object
{
MotionComputationWorker::MotionComputationWorker(PublishObjectCallback obj_pub) : obj_pub_(obj_pub){};

void MotionComputationWorker::detectedObjectCallback(const autoware_msgs::DetectedObjectArray& obj_array)
{
  cav_msgs::ExternalObjectList msg;
  msg.header = obj_array.header;

  for (int i = 0; i < obj_array.objects.size(); i++)
  {
    cav_msgs::ExternalObject obj;

    // Header contains the frame rest of the fields will use
    obj.header = obj_array.objects[i].header;

    // Presence vector message is used to describe objects coming from potentially
    // different sources. The presence vector is used to determine what items are set
    // by the producer
    obj.presence_vector = obj.presence_vector | obj.ID_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.POSE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.VELOCITY_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.SIZE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.OBJECT_TYPE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.DYNAMIC_OBJ_PRESENCE;

    // Object id. Matching ids on a topic should refer to the same object within some time period, expanded
    obj.id = obj_array.objects[i].id;

    // Pose of the object within the frame specified in header
    obj.pose.pose = obj_array.objects[i].pose;
    obj.pose.covariance[0] = obj_array.objects[i].variance.x;
    obj.pose.covariance[7] = obj_array.objects[i].variance.y;
    obj.pose.covariance[17] = obj_array.objects[i].variance.z;

    // Average velocity of the object within the frame specified in header
    obj.velocity.twist = obj_array.objects[i].velocity;

    // The size of the object aligned along the axis of the object described by the orientation in pose
    // Dimensions are specified in meters
    obj.size = obj_array.objects[i].dimensions;

    // Update the object type and generate predictions using CV or CTRV vehicle models.
		// If the object is a bicycle or motor vehicle use CTRV otherwise use CV.
    bool use_ctrv_model = true;

    if (obj_array.objects[i].label.compare("person"))
    {
      obj.object_type = obj.PEDESTRIAN;
      use_ctrv_model = false;
    }
    else
    {
      obj.object_type = obj.UNKNOWN;
      use_ctrv_model = false;
    }

    if (use_ctrv_model)
    {
      obj.predictions =
          motion_predict::ctrv::predictPeriod(obj, prediction_time_step_, prediction_period_,
                                              prediction_process_noise_max_, prediction_confidence_drop_rate_);
    }
    else
    {
      obj.predictions = motion_predict::cv::predictPeriod(
          obj, prediction_time_step_, prediction_period_, cv_x_accel_noise_, cv_y_accel_noise_,
          prediction_process_noise_max_, prediction_confidence_drop_rate_);
    }

    // Binary value to show if the object is static or dynamic (1: dynamic, 0: static)

    if ((abs(obj.velocity.twist.linear.x || obj.velocity.twist.linear.y || obj.velocity.twist.linear.z)) > 0.75)
    {
      obj.dynamic_obj = 1;
    }
    else
    {
      obj.dynamic_obj = 0;
    }

    msg.objects.emplace_back(obj);
  }

  obj_pub_(msg);
}


}  // namespace object
