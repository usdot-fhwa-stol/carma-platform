/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <carma_wm/TrafficControl.h>

namespace carma_wm
{

void toBinMsg(std::shared_ptr<carma_wm::TrafficControl> gf_ptr, autoware_lanelet2_msgs::MapBin* msg)
{
  if (msg == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": msg is null pointer!");
    return;
  }
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *gf_ptr;
  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void fromBinMsg(const autoware_lanelet2_msgs::MapBin& msg, std::shared_ptr<carma_wm::TrafficControl>& gf_ptr)
{
  auto gf_copy = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  
  if (!gf_ptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": gf_ptr is null pointer!");
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());
  
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);

  oa >> *gf_copy;
  gf_ptr = gf_copy;
  ROS_ERROR_STREAM("GFCOPY USE COUNT" << gf_copy.use_count());

  ROS_DEBUG_STREAM("This is the last one UPDATE: Check if the constData has a problem!:");
  for (auto params : gf_ptr->update_list_.front().second->constData()->parameters)
  {
    ROS_DEBUG_STREAM("param: " << params.first);

    for (auto& param : params.second)
    {
      //auto l = boost::get<lanelet::ConstLanelet>(&param);
      //if (l != nullptr)
      //{
      //ROS_ERROR_STREAM("llt: id " << l->id());
      //}
      auto weak = boost::get<lanelet::WeakLanelet>(&param);
      if (weak == nullptr)
      {
        ROS_ERROR_STREAM("Not working");
      }
      else
      {
        if (weak->expired())
        {
          ROS_ERROR_STREAM("FROM BIN Sadly expired...");
        }
        //auto llt = weak->lock();
        //ROS_ERROR_STREAM("WOW IT WORKED FOR THIS???? llt " << llt.id());
        std::vector<lanelet::WeakLanelet> weak_list;
        weak_list.push_back(*weak);
        ROS_ERROR_STREAM("Count of weak pointer: " << weak_list.front().laneletData_.use_count());
        auto strong_list = lanelet::utils::strong(weak_list);
        ROS_ERROR_STREAM("FROM BIN size: " << strong_list.size());
      }
    }
  }

  
}

}  // namespace carma_wm
