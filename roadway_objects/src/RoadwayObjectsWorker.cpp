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
#include "RoadwayObjectsWorker.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/utility/Optional.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// TODO whole file

namespace objects{

RoadwayObjectsWorker::RoadwayObjectsWorker(carma_wm::WorldModelConstPtr wm, PublishObstaclesCallback obj_pub):obj_pub_(obj_pub), wm_(wm) {}

void RoadwayObjectsWorker::externalObjectsCallback(const cav_msgs::ExternalObjectListConstPtr& obj_array)
{	
	cav_msgs::RoadwayObstacleList obstacle_list;
	auto map = wm_->getMap();
	if (!map) {
		ROS_WARN("roadway_objects could not process external objects as no semantic map was available");
		return;
	}

	if (map->laneletLayer.size() == 0) {
		ROS_WARN("roadway_objects could not process external objects as the semantic map does not contain any lanelets");
		return;
	}

	// 1. Find matching lanelet of each object in list

	for(auto object : obj_array->objects) {

		tf2::Transform object_tf;
		tf2::fromMsg(object.pose.pose, object_tf);

		double half_x_bound = object.size.x / 2;
		double half_y_bound = object.size.y / 2;

		// 4 corners of the object starting with upper left and moving in clockwise direction in pose frame
		tf2::Vector3 obj_p1(half_x_bound, -half_y_bound, object.pose.pose.position.z);
		tf2::Vector3 obj_p2(half_x_bound, half_y_bound, object.pose.pose.position.z);
		tf2::Vector3 obj_p3(-half_x_bound, -half_y_bound, object.pose.pose.position.z);
		tf2::Vector3 obj_p4(-half_x_bound, half_y_bound, object.pose.pose.position.z);

		tf2::Vector3 obj_p1_map = object_tf * obj_p1;
		tf2::Vector3 obj_p2_map = object_tf * obj_p2;
		tf2::Vector3 obj_p3_map = object_tf * obj_p3;
		tf2::Vector3 obj_p4_map = object_tf * obj_p4;

		lanelet::BasicPoint2d p1(obj_p1_map.getX(), obj_p1_map.getY());
		lanelet::BasicPoint2d p2(obj_p2_map.getX(), obj_p2_map.getY());
		lanelet::BasicPoint2d p3(obj_p3_map.getX(), obj_p3_map.getY());
		lanelet::BasicPoint2d p4(obj_p4_map.getX(), obj_p4_map.getY());

		lanelet::BasicPoint3d object_center(object_tf.getOrigin().getX(), object_tf.getOrigin().getY(), object_tf.getOrigin().getZ());
		lanelet::BasicPolygon2d object_polygon = {p1, p2, p3, p4};

		auto nearestLanelet = map->laneletLayer.nearest(lanelet::utils::to2D(object_center), 1)[0]; // Since the map contains lanelets there should always be at least 1 element
		
		// Check if the object is inside 
		lanelet::BasicPolygon2d lanelet_polygon = nearestLanelet.polygon2d().basicPolygon();
    	bool val = boost::geometry::intersects(lanelet_polygon, object_polygon); 
		
		// if (!lanelet::geometry::inside(nearestLanelet, object_center)) {
		// 	// 
		// }
	}


	// 2. Check if object is within lanelet. If not ignore object
	// 3. For remaining objects record downtrack/crosstrack
	// 4. Determine prediction for next lanelet information


	obj_pub_(obstacle_list);
}

}//object
