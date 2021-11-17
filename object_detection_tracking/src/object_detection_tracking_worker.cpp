/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include "object_detection_tracking_worker.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>


namespace object
{
ObjectDetectionTrackingWorker::ObjectDetectionTrackingWorker(PublishObjectCallback obj_pub) : obj_pub_(obj_pub){};

bool isClass(const autoware_auto_msgs::msg::TrackedObject& obj, uint8_t class_id) {

  return obj.classification.end() != std::find_if(obj.classification.begin(), obj.classification.end(), 
    [&class](auto o){ return o.classification == class_id; }
  );

}

void ObjectDetectionTrackingWorker::detectedObjectCallback(const autoware_msgs::DetectedObjectArray& obj_array)
{

  carma_perception_msgs::msg::ExternalObjectList msg;
  msg.header = obj_array.header;
  msg.header.frame_id = map_frame_;


  geometry_msgs::TransformStamped object_frame_tf; 

  try {

    object_frame_tf = tfBuffer_.lookupTransform(map_frame_ ,obj_array.header.frame_id, obj_array.header.stamp);

  } catch (tf2::TransformException &ex) {

    ROS_WARN_STREAM("Ignoring fix message: Could not locate static transforms with exception " << ex.what());
    return;
  }

  for (int i = 0; i < obj_array.objects.size(); i++)
  {
    carma_perception_msgs::msg::ExternalObject obj;

    // Header contains the frame rest of the fields will use
    obj.header = obj_array.objects[i].header;
    obj.header.frame_id = map_frame_;

    // Presence vector message is used to describe objects coming from potentially
    // different sources. The presence vector is used to determine what items are set
    // by the producer
    obj.presence_vector = obj.presence_vector | obj.ID_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.POSE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.VELOCITY_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.SIZE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.OBJECT_TYPE_PRESENCE_VECTOR;
    obj.presence_vector = obj.presence_vector | obj.DYNAMIC_OBJ_PRESENCE;
    obj.presence_vector = obj.presence_vector | obj.CONFIDENCE_PRESENCE_VECTOR;
     
    // Object id. Matching ids on a topic should refer to the same object within some time period, expanded
    obj.id = obj_array.objects[i].object_id;

    // Pose of the object within the frame specified in header
    geometry_msgs::msg::Pose input_object_pose;
    input_object_pose.position = obj_array.objects[i].kinematics.centroid_position;
    input_object_pose.orientation = obj_array.objects[i].kinematics.orientation;

    tf2::doTransform(input_object_pose, obj.pose.pose, object_frame_tf);


    obj.confidence = obj_array.objects[i].existence_probability;

    // Map the 3x3 matrix from autoware to the upper 3x3 matrix in geometry_msgs 
    // TODO need to investigate the impact of the coordinate transformation on the covariance matrix
    obj.pose.covariance[0] = obj_array.objects[i].position_covariance[0]
    obj.pose.covariance[1] = obj_array.objects[i].position_covariance[1]
    obj.pose.covariance[2] = obj_array.objects[i].position_covariance[2]
    obj.pose.covariance[6] = obj_array.objects[i].position_covariance[3]
    obj.pose.covariance[7] = obj_array.objects[i].position_covariance[4]
    obj.pose.covariance[8] = obj_array.objects[i].position_covariance[5]
    obj.pose.covariance[12] = obj_array.objects[i].position_covariance[6]
    obj.pose.covariance[13] = obj_array.objects[i].position_covariance[7]
    obj.pose.covariance[14] = obj_array.objects[i].position_covariance[8]

    

    // Average velocity of the object within the frame specified in header
    obj.velocity.twist = obj_array.objects[i].kinematics.twist;

    // The size of the object aligned along the axis of the object described by the orientation in pose
    // Dimensions are specified in meters
    // Autoware provides a set of 2d bounding boxes and supports articulated shapes.
    // The loop below finds the overall bounding box to ensure information is not lost

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for(auto polygon : obj_array.objects[i].shape) {
      for (auto point :  polygon) {
        
        if (point.x > maxX)
          maxX = point.x;

        if (point.x < minX)
          minX = point.x;

        if (point.y > maxY)
          maxY = point.y;

        if (point.y < minY)
          minY = point.y;
      }
    }

    double dX = maxX - minX;
    double dY = maxY - minY;

    obj.size.x = dX / 2.0; // Shape in carma is defined by the half delta from the centroid
    obj.size.y = dY / 2.0;

    // Height provided by autoware is overall height divide by 2 for delta from centroid
    obj.size.z = obj_array.objects[i].shape.height / 2.0; 

    // Update the object type and generate predictions using CV or CTRV vehicle models.
		// If the object is a bicycle or motor vehicle use CTRV otherwise use CV.

    if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::MOTORCYCLE))
    {
      obj.object_type = obj.MOTORCYCLE;
    }
    else if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::BICYCLE))
    {
      obj.object_type = obj.MOTORCYCLE; // Currently external object cannot represent bicycles
    }
    else if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::CAR))
    {
      obj.object_type = obj.SMALL_VEHICLE;
    }
    else if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::TRUCK))
    {
      obj.object_type = obj.LARGE_VEHICLE;
    }
    else if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::TRAILER))
    {
      obj.object_type = obj.LARGE_VEHICLE; // Currently external object cannot represent trailers
    }
    else if (isClass(obj_array.objects[i], autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN))
    {
      obj.object_type = obj.PEDESTRIAN;
    }
    else
    {
      obj.object_type = obj.UNKNOWN;
    }

    // Binary value to show if the object is static or dynamic (1: dynamic, 0: static)

    if ((fabs(obj.velocity.twist.linear.x || obj.velocity.twist.linear.y || obj.velocity.twist.linear.z)) > 0.75)
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

void ObjectDetectionTrackingWorker::setMapFrame(std::string map_frame)
{
  map_frame_ = map_frame;
}


}  // namespace object
