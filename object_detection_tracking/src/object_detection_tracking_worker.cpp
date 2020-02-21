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
#include "object_detection_tracking_worker.h"

namespace object{

void ObjectDetectionTrackingWorker::detectedObjectCallback(const autoware_msgs::DetectedObjectArray &obj_array);
{	
	
	cav_msgs::ExternalObjectList msg;

	for(int i=0;i<obj_array.size();i++)
	{
		cav_msgs::ExternalObject obj;

		//Header contains the frame rest of the fields will use
		 obj.header=obj_array[i].header;

		//Presence vector message is used to describe objects coming from potentially
		//different sources. The presence vector is used to determine what items are set
		//by the producer
		obj.presence_vector=obj.presence_vector || ID_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || POSE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || VELOCITY_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || VELOCITY_INST_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || SIZE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || CONFIDENCE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || OBJECT_TYPE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || BSM_ID_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector || DYNAMIC_OBJ_PRESENCE;
		obj.presence_vector=obj.presence_vector || PREDICTION_PRESENCE_VECTOR;



		//Object id. Matching ids on a topic should refer to the same object within some time period, expanded
		obj.id=obj_array[i].id;



		//Pose of the object within the frame specified in header
		obj.pose=obj_array[i].pose;

		//Average velocity of the object within the frame specified in header
		obj.velocity=obj_array[i].velocity;


		//The size of the object aligned along the axis of the object described by the orientation in pose
		//Dimensions are specified in meters
		//obj.size=

		//Confidence [0,1]
		//obj.confidence=

		//describes a general object type as defined in this message
        object_type=UNKNOWN;
/*
#used for object type
uint8 UNKNOWN = 0
uint8 SMALL_VEHICLE = 1
uint8 LARGE_VEHICLE = 2
uint8 MOTORCYCLE = 3
uint8 PEDESTRIAN = 4

# Binary value to show if the object is static or dynamic (1: dynamic, 0: static)
bool dynamic_obj

#Predictions for the object
cav_msgs/PredictedState[] prediction*/



		
		msg.emplace_back(obj);
	}

	pub_object_.publish(msg);
}

void set_publishers(ros::Publisher pub_object)
{
	pub_object_=pub_object;
}

}