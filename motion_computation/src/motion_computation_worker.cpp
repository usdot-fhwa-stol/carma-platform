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

void MotionComputationWorker::motionPredictionCallback(const autoware_msgs::DetectedObjectArray& dup_obj_array)//Duplicate input message
{
  cav_msgs::ExternalObjectList msg;
  msg.header = dup_obj_array.header;

  for (int i = 0; i < dup_obj_array.objects.size(); i++)
  {
    cav_msgs::ExternalObject obj;

    // Header contains the frame rest of the fields will use
    obj.header = dup_obj_array.objects[i].header;

    // Object id. Matching ids on a topic should refer to the same object within some time period, expanded
    obj.id = dup_obj_array.objects[i].id;

    // Update the object type and generate predictions using CV or CTRV vehicle models.
		// If the object is a bicycle or motor vehicle use CTRV otherwise use CV.

    bool use_ctrv_model;

    /*if (  obj.object_type == obj.UNKNOWN)
    {
      use_ctrv_model = true;

    }
    else if (obj.object_type == obj.MOTORCYCLE)
    {
      use_ctrv_model = true;

    }
    else if (obj.object_type == obj.SMALL_VEHICLE)
    {
      use_ctrv_model = true;

    }
    else if (obj.object_type == obj.LARGE_VEHICLE;)
    {
      use_ctrv_model = true;

    }

    else if ( obj.object_type == obj.PEDESTRIAN)
    {
      use_ctrv_model = false;
    }
    else
    {
      //obj.object_type = obj.UNKNOWN;
      use_ctrv_model = false;
    }//end if-else */

    //Switch Statement Test
    switch(obj.object_type)
    {
      case obj.UNKNOWN:
        use_ctrv_model = true;

      case obj.MOTORCYCLE:
        use_ctrv_model = true;

      case obj.SMALL_VEHICLE:
        use_ctrv_model = true;
      
      case obj.LARGE_VEHICLE:
        use_ctrv_model = true;
      
      case obj.PEDESTRIAN:
        use_ctrv_model = false;

      default:
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

     msg.objects.emplace_back(obj);

   
  }
  
   obj_pub_(msg);

}

}  // namespace object
