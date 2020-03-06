/*
 * Copyright (C) 2020 LEIDOS.
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

#include <carma_wm_ctrl/WMBroadcaster.h>
#include <carma_wm_ctrl/ROSTimerFactory.h>
#include <carma_wm_ctrl/WMBroadcasterNode.h>

namespace carma_wm_ctrl
{
using std::placeholders::_1;

void WMBroadcasterNode::publishMap(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  map_pub_.publish(map_msg);
}

WMBroadcasterNode::WMBroadcasterNode()
  : wmb_(std::bind(&WMBroadcasterNode::publishMap, this, _1), std::make_unique<ROSTimerFactory>()){};

int WMBroadcasterNode::run()
{
  // Map Publisher
  map_pub_ = cnh_.advertise<autoware_lanelet2_msgs::MapBin>("semantic_map", 1, true);
  // Base Map Sub
  base_map_sub_ = cnh_.subscribe("base_map", 1, &WMBroadcaster::baseMapCallback, &wmb_);

  // Spin
  cnh_.setSpinRate(10);
  cnh_.spin();
  return 0;
}

}  // namespace carma_wm_ctrl
