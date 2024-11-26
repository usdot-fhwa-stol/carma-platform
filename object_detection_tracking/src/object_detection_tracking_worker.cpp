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
#include "covariance_helper.h"

namespace object
{

ObjectDetectionTrackingWorker::ObjectDetectionTrackingWorker(PublishObjectCallback obj_pub, TransformLookupCallback tf_lookup, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger) 
  : obj_pub_(obj_pub), tf_lookup_(tf_lookup), logger_(logger) {}

bool ObjectDetectionTrackingWorker::isClass(const autoware_auto_msgs::msg::TrackedObject& obj, uint8_t class_id) {

  return obj.classification.end() != std::find_if(obj.classification.begin(), obj.classification.end(), 
    [&class_id](auto o){ return o.classification == class_id; }
  );

}

void ObjectDetectionTrackingWorker::detectedObjectCallback(autoware_auto_msgs::msg::TrackedObjects::UniquePtr  obj_array)
{

  carma_perception_msgs::msg::ExternalObjectList msg;
  msg.header = obj_array->header;
  msg.header.frame_id = map_frame_;


  auto transform = tf_lookup_(map_frame_, obj_array->header.frame_id, obj_array->header.stamp);

  if (!transform) {
    RCLCPP_WARN_STREAM(logger_->get_logger(), "Ignoring fix message: Could not locate static transforms exception.");
    return;
  }

  geometry_msgs::msg::TransformStamped object_frame_tf = transform.get(); 

  for (size_t i = 0; i < obj_array->objects.size(); i++)
  {
    carma_perception_msgs::msg::ExternalObject obj;

    // Header contains the frame rest of the fields will use
    obj.header = msg.header;

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
    obj.id = obj_array->objects[i].object_id;

    // Pose of the object within the frame specified in header
    geometry_msgs::msg::PoseStamped input_object_pose;
    input_object_pose.header = obj_array->header;
    input_object_pose.pose.position = obj_array->objects[i].kinematics.centroid_position;
    input_object_pose.pose.orientation = obj_array->objects[i].kinematics.orientation;

    geometry_msgs::msg::PoseStamped output_pose;
    // Transform the object input our map frame
    tf2::doTransform(input_object_pose, output_pose, object_frame_tf);

    obj.pose.pose = output_pose.pose;

    // In ROS2 foxy the doTransform call does not set the covariance, so we need to do it manually
    // Copy over the position covariance
    // Variable names used here are row, column so row major order
    auto xx = obj_array->objects[i].kinematics.position_covariance[0];
    auto xy = obj_array->objects[i].kinematics.position_covariance[1];
    auto xz = obj_array->objects[i].kinematics.position_covariance[2];
    auto yx = obj_array->objects[i].kinematics.position_covariance[3];
    auto yy = obj_array->objects[i].kinematics.position_covariance[4];
    auto yz = obj_array->objects[i].kinematics.position_covariance[5];
    auto zx = obj_array->objects[i].kinematics.position_covariance[6];
    auto zy = obj_array->objects[i].kinematics.position_covariance[7];
    auto zz = obj_array->objects[i].kinematics.position_covariance[8];

    // This matrix represents the covariance of the object before transformation
    std::array<double, 36> input_covariance = { 
      xx, xy, xz,  0, 0, 0,
      yx, yy, yz,  0, 0, 0,
      zx, zy, zz,  0, 0, 0,
      0,  0,  0,  1,  0, 0, // Since no covariance for the orientation is provided we will assume an identity relationship (1s on the diagonal)
      0,  0,  0,  0,  1, 0, // TODO when autoware suplies this information we should update this to reflect the new covariance
      0,  0,  0,  0,  0, 1
    };

    // Transform the covariance matrix
    tf2::Transform covariance_transform;
    tf2::fromMsg(object_frame_tf.transform, covariance_transform);
    obj.pose.covariance = covariance_helper::transformCovariance(input_covariance, covariance_transform);

    // Store the object ovarall confidence
    obj.confidence = obj_array->objects[i].existence_probability;

    // Average velocity of the object within the frame specified in header
    obj.velocity = obj_array->objects[i].kinematics.twist;

    // The size of the object aligned along the axis of the object described by the orientation in pose
    // Dimensions are specified in meters
    // Autoware provides a set of 2d bounding boxes and supports articulated shapes.
    // The loop below finds the overall bounding box to ensure information is not lost

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    double maxHeight = std::numeric_limits<double>::lowest();

    for(auto shape : obj_array->objects[i].shape) {
      for (auto point :  shape.polygon.points) {
        
        if (point.x > maxX)
          maxX = point.x;

        if (point.x < minX)
          minX = point.x;

        if (point.y > maxY)
          maxY = point.y;

        if (point.y < minY)
          minY = point.y;
      }
      if (shape.height > maxHeight)
        maxHeight = shape.height;
    }

    double dX = maxX - minX;
    double dY = maxY - minY;

    obj.size.x = dX / 2.0; // Shape in carma is defined by the half delta from the centroid
    obj.size.y = dY / 2.0;

    // Height provided by autoware is overall height divide by 2 for delta from centroid
    obj.size.z = maxHeight / 2.0; 

    // Update the object type and generate predictions using CV or CTRV vehicle models.
		// If the object is a bicycle or motor vehicle use CTRV otherwise use CV.

    if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::MOTORCYCLE))
    {
      obj.object_type = obj.MOTORCYCLE;
    }
    else if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::BICYCLE))
    {
      obj.object_type = obj.MOTORCYCLE; // Currently external object cannot represent bicycles
    }
    else if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::CAR))
    {
      obj.object_type = obj.SMALL_VEHICLE;
    }
    else if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::TRUCK))
    {
      obj.object_type = obj.LARGE_VEHICLE;
    }
    else if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::TRAILER))
    {
      obj.object_type = obj.LARGE_VEHICLE; // Currently external object cannot represent trailers
    }
    else if (isClass(obj_array->objects[i], autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN))
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
