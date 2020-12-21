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

#include "traffic_incident_worker.h"
#include <gtest/gtest.h>

namespace traffic
{

TEST(TrafficIncidentWorkerTest, testTrafficMobilityOperationBroadcastStrategyParams)
{

    TrafficIncidentWorker traffic_worker([](auto msg){});
  
    gps_common::GPSFix msg;
   
    msg.latitude=57.1;
    msg.longitude=155.79;
    msg.header.stamp.sec=25;

    std::string sender_id="USDOT-49096";
    std::string closed_lane="[1]";
    double down_track=50.1;
    double up_track=50.1;
    
    traffic_worker.setSenderId(sender_id);
    traffic_worker.setClosedLane(closed_lane);
    traffic_worker.setDownTrack(down_track);
    traffic_worker.setUpTrack(up_track);

    cav_msgs::MobilityOperation traffic_msg=traffic_worker.mobilityMessageGenerator(msg);
 
    EXPECT_EQ(traffic_msg.strategy_params,"lat:57.1,lon:155.79,closed_lanes:[1],downtrack:50.1,uptrack:50.1");
  
  }

  TEST(TrafficIncidentWorkerTest, testTrafficMobilityOperationBroadcastTimeStamp)
{

    TrafficIncidentWorker traffic_worker([](auto msg){});
  
    gps_common::GPSFix msg;
   
    msg.latitude=57.1;
    msg.longitude=155.79;
    msg.header.stamp.sec=25;

    std::string sender_id="USDOT-49096";
    std::string closed_lane="[1]";
    double down_track=50.1;
    double up_track=50.1;
    
    traffic_worker.setSenderId(sender_id);
    traffic_worker.setClosedLane(closed_lane);
    traffic_worker.setDownTrack(down_track);
    traffic_worker.setUpTrack(up_track);

    cav_msgs::MobilityOperation traffic_msg=traffic_worker.mobilityMessageGenerator(msg);
 
    EXPECT_EQ(traffic_msg.header.timestamp,25000);
 
  }

  TEST(TrafficIncidentWorkerTest, testTrafficMobilityOperationBroadcastStrategy)
{

    TrafficIncidentWorker traffic_worker([](auto msg){});
  
    gps_common::GPSFix msg;
   
    msg.latitude=57.1;
    msg.longitude=155.79;
    msg.header.stamp.sec=25;

    std::string sender_id="USDOT-49096";
    std::string closed_lane="[1]";
    double down_track=50.1;
    double up_track=50.1;
    
    traffic_worker.setSenderId(sender_id);
    traffic_worker.setClosedLane(closed_lane);
    traffic_worker.setDownTrack(down_track);
    traffic_worker.setUpTrack(up_track);

    cav_msgs::MobilityOperation traffic_msg=traffic_worker.mobilityMessageGenerator(msg);
 
    EXPECT_EQ(traffic_msg.header.sender_id,"USDOT-49096");
    EXPECT_EQ(traffic_msg.strategy,"carma3/Incident_Use_Case");
  
  }
  
    TEST(TrafficIncidentWorkerTest, testAnyTypeToStringFunction)
{
   TrafficIncidentWorker traffic_worker([](auto msg){});
 
   EXPECT_EQ(traffic_worker.anytypeToString(55.6712),"55.6712");
}

}//traffic
