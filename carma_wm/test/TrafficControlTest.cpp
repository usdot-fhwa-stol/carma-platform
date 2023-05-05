/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <carma_wm/TrafficControl.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include "TestHelpers.hpp"

#include <carma_v2x_msgs/msg/traffic_control_message.hpp>


namespace carma_wm

{

TEST(TrafficControl, TrafficControlBinMsgTest)
{
  using namespace lanelet::units::literals;
  // add a lanelet
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 0, 0);

  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p4, p3 });

  auto ll_1 = getLanelet(left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);
  // add regems

  lanelet::DigitalSpeedLimitPtr speed_limit_old = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(9000, 5_mph, {ll_1}, {},
                                                     { lanelet::Participants::VehicleCar }));
  lanelet::DigitalSpeedLimitPtr speed_limit_new = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(9001, 5_mph, {ll_1}, {},
                                                     { lanelet::Participants::VehicleCar }));

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  gf_ptr->id_ = boost::uuids::random_generator()();

  gf_ptr->remove_list_.push_back(std::make_pair(ll_1.id(), speed_limit_old));
  gf_ptr->update_list_.push_back(std::make_pair(ll_1.id(), speed_limit_new));

  // from broadcaster
  autoware_lanelet2_msgs::msg::MapBin gf_obj_msg;
  
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_, {ll_1}));
  RCLCPP_INFO(rclcpp::get_logger("carma_wm::TrafficControlTest"), "Below null pointer error message is expected");
  auto null = nullptr;
  carma_wm::toBinMsg(send_data, null);
  ASSERT_EQ(null, nullptr);
  carma_wm::toBinMsg(send_data, &gf_obj_msg);
  
  // at map users
  auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  RCLCPP_INFO(rclcpp::get_logger("carma_wm::TrafficControlTest"), "Below null pointer error message is expected");
  carma_wm::fromBinMsg(gf_obj_msg, null);
  ASSERT_EQ(null, nullptr);
  carma_wm::fromBinMsg(gf_obj_msg, data_received);

  ASSERT_EQ(data_received->id_, gf_ptr->id_); // same element
  ASSERT_EQ(gf_ptr->remove_list_.size(), 1);
  ASSERT_EQ(data_received->remove_list_.size(), 1); // old_speed_limit
  ASSERT_EQ(data_received->remove_list_[0].second->attribute(lanelet::AttributeName::Subtype).value(), lanelet::DigitalSpeedLimit::RuleName );
  ASSERT_EQ(data_received->update_list_.size(), 1); // new_speed_limit
  ASSERT_EQ(data_received->update_list_[0].first, gf_ptr->update_list_[0].first);
  ASSERT_NE(data_received->update_list_[0].second, gf_ptr->update_list_[0].second); // they are now not same because of serialization, the data address is different
                                                                                    // but again, they are same elements
  ASSERT_EQ(data_received->lanelet_additions_.size(), 1);
  ASSERT_EQ(data_received->lanelet_additions_[0].id(), ll_1.id());

}

}  // namespace carma_wm_ctrl