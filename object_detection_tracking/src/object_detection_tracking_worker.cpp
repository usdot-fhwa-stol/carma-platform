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
	msg.header=obj_array.header;

	for(int i=0;i<obj_array.objects.size();i++)
	{
		cav_msgs::ExternalObject obj;

		//Header contains the frame rest of the fields will use
		 obj.header=obj_array.objects[i].header;

		//Presence vector message is used to describe objects coming from potentially
		//different sources. The presence vector is used to determine what items are set
		//by the producer
		obj.presence_vector=obj.presence_vector | ID_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector | POSE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector | VELOCITY_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector | SIZE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector | OBJECT_TYPE_PRESENCE_VECTOR;
		obj.presence_vector=obj.presence_vector | DYNAMIC_OBJ_PRESENCE;
		
		//Object id. Matching ids on a topic should refer to the same object within some time period, expanded
		obj.id=obj_array.objects[i].id;

		//Pose of the object within the frame specified in header
		obj.pose.pose=obj_array.objects[i].pose;
		obj.pose.covariance[0]=obj_array.objects[i].variance[0];
		obj.pose.covariance[7]=obj_array.objects[i].variance[1];
		obj.pose.covariance[17]=obj_array.objects[i].variance[2];

		//Average velocity of the object within the frame specified in header
		obj.velocity.twist=obj_array.objects[i].velocity;

		//The size of the object aligned along the axis of the object described by the orientation in pose
		//Dimensions are specified in meters
		for(int j=0;j<obj_array.objects[i].dimensions.size();j++)
		{
		obj.size[j]=obj_array.objects[i].dimensions[j];
	    }

		//describes a general object type as defined in this message
        obj.object_type=UNKNOWN;

		// Binary value to show if the object is static or dynamic (1: dynamic, 0: static)
		
        if( (abs(obj.velocity.twist.linear.x || obj.velocity.twist.linear.x || obj.velocity.twist.linear.x)) > 0 )
        {
		  obj.dynamic_obj=1;
		}
		else
		{
		  obj.dynamic_obj=0;
		}
		
		msg.objects.emplace_back(obj);
	}

		pub_object_.publish(msg);
}

void set_publishers(ros::Publisher pub_object)
{
	pub_object_=pub_object;
}

}