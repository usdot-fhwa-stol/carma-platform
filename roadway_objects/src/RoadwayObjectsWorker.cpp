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

// TODO whole file

namespace objects{

RoadwayObjectsWorker::RoadwayObjectsWorker(PublishObstaclesCallback obj_pub):obj_pub_(obj_pub) { };

void RoadwayObjectsWorker::externalObjectsCallback(const cav_msgs::ExternalObjectList &obj_array)
{	
	cav_msgs::RoadwayObstacleList obstacle_list;

	obj_pub_(obstacle_list);
}

}//object
