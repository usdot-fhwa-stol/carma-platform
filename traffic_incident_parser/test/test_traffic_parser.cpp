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

#include "traffic_incident_parser_worker.h"
#include <gtest/gtest.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>

namespace traffic
{

TEST(TrafficIncidentParserWorkerTest, testMobilityMessageParser1)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
    
  std::string mobility_strategy_params="lat:0.435,lon:0.555,downtrack:5,uptrack:5,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW,event_type:CLOSED";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);
  
  EXPECT_EQ(traffic_worker.latitude,0.435);
  EXPECT_EQ(traffic_worker.longitude,0.555);
  EXPECT_EQ(traffic_worker.down_track,5);
  EXPECT_EQ(traffic_worker.up_track,5);  
  EXPECT_EQ(traffic_worker.min_gap,2);
  EXPECT_EQ(traffic_worker.speed_advisory,1.2);
  EXPECT_EQ(traffic_worker.event_reason,"MOVE OVER LAW");
  EXPECT_EQ(traffic_worker.event_type,"CLOSED");
  
  }


TEST(TrafficIncidentParserWorkerTest, testMobilityMessageParser2)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
      
  std::string mobility_strategy_params="lat:0.75,lon:0.555,downtrack:75,uptrack:55,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW3,event_type:OPEN";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);
  
  EXPECT_EQ(traffic_worker.latitude,0.75);
  EXPECT_EQ(traffic_worker.longitude,0.555);
  EXPECT_EQ(traffic_worker.down_track,75);
  EXPECT_EQ(traffic_worker.up_track,55);
  EXPECT_EQ(traffic_worker.min_gap,2);  
  EXPECT_EQ(traffic_worker.speed_advisory,1.2);
  EXPECT_EQ(traffic_worker.event_reason,"MOVE OVER LAW3");
  EXPECT_EQ(traffic_worker.event_type,"OPEN");
  
  }


TEST(TrafficIncidentParserWorkerTest, testMobilityMessageParser3)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
   
  std::string mobility_strategy_params="lat:0.3,lon:0.95,downtrack:57,uptrack:59,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW,event_type:CLOSED";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);
  
  EXPECT_EQ(traffic_worker.latitude,0.3);
  EXPECT_EQ(traffic_worker.longitude,0.95);
  EXPECT_EQ(traffic_worker.down_track,57);
  EXPECT_EQ(traffic_worker.up_track,59);
  EXPECT_EQ(traffic_worker.min_gap,2); 
  EXPECT_EQ(traffic_worker.speed_advisory,1.2);
  EXPECT_EQ(traffic_worker.event_reason,"MOVE OVER LAW");
  EXPECT_EQ(traffic_worker.event_type,"CLOSED");
  
  }

  TEST(TrafficIncidentParserWorkerTest, earthToMapFrame)
{

  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
  std_msgs::String projection_msg;
  projection_msg.data="+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";

  traffic_worker.projectionCallback(projection_msg);

  std::string mobility_strategy_params="lat:39.46636844371259,lon:-76.16919523566943,downtrack:57,uptrack:59,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW,event_type:CLOSED";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);

  lanelet::BasicPoint2d local_point=traffic_worker.getIncidentOriginPoint();
    
  EXPECT_NEAR(local_point.x(),0,0.001);
  EXPECT_NEAR(local_point.y(),0,0.001);
}

  TEST(TrafficIncidentParserWorkerTest, composeTrafficControlMesssage)
{
  ros::Time::init();
  auto cmw= carma_wm::test::getGuidanceTestMap();
  carma_wm::test::setRouteByIds({1200, 1201,1202,1203}, cmw);
  
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
  std_msgs::String projection_msg;
  projection_msg.data="+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";

  traffic_worker.projectionCallback(projection_msg);

  std::string mobility_strategy_params="lat:39.46636844371259,lon:-76.16919523566943,downtrack:99,uptrack:25,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW,event_type:CLOSED";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);
  
  std::vector<cav_msgs::TrafficControlMessageV01> traffic_mobility_msg_test=traffic_worker.composeTrafficControlMesssages();

  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[0].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[0].y,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[1].x,3.7,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[1].y,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[2].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[2].y,25.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[3].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[3].y,25.0,0.00);

  EXPECT_EQ(traffic_mobility_msg_test[0].geometry_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].params_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].package_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].params.detail.choice,cav_msgs::TrafficControlDetail::CLOSED_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[0].params.detail.closed,cav_msgs::TrafficControlDetail::CLOSED);
  EXPECT_EQ(traffic_mobility_msg_test[0].package.label,"MOVE OVER LAW");

  EXPECT_EQ(traffic_mobility_msg_test[1].params.detail.choice,cav_msgs::TrafficControlDetail::MINHDWY_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[1].params.detail.minhdwy,2);

  EXPECT_EQ(traffic_mobility_msg_test[2].params.detail.choice,cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[2].params.detail.maxspeed,1.2);

}

 TEST(TrafficIncidentParserWorkerTest, composeTrafficControlMesssage1)
{
  ros::Time::init();
  auto cmw= carma_wm::test::getGuidanceTestMap();
  carma_wm::test::setRouteByIds({1200, 1201,1202,1203}, cmw);
  
  TrafficIncidentParserWorker traffic_worker(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),[](auto msg){});
  std_msgs::String projection_msg;
  projection_msg.data="+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
	
  traffic_worker.projectionCallback(projection_msg);

  std::string mobility_strategy_params="lat:39.46663865458896225,lon:-76.16919523566940597,downtrack:99,uptrack:25,min_gap:2,advisory_speed:1.2,event_reason:MOVE OVER LAW,event_type:CLOSED";
  traffic_worker.mobilityMessageParser(mobility_strategy_params);
  
  std::vector<cav_msgs::TrafficControlMessageV01> traffic_mobility_msg_test=traffic_worker.composeTrafficControlMesssages();

  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[0].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[0].y,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[1].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[1].y,25.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[2].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[2].y,25.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[3].x,0.0,0.001);
  EXPECT_NEAR(traffic_mobility_msg_test[0].geometry.nodes[3].y,25.0,0.001);

  EXPECT_EQ(traffic_mobility_msg_test[0].geometry_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].params_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].package_exists,true);
  EXPECT_EQ(traffic_mobility_msg_test[0].params.detail.choice,cav_msgs::TrafficControlDetail::CLOSED_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[0].params.detail.closed,cav_msgs::TrafficControlDetail::CLOSED);
  EXPECT_EQ(traffic_mobility_msg_test[0].package.label,"MOVE OVER LAW");

  EXPECT_EQ(traffic_mobility_msg_test[1].params.detail.choice,cav_msgs::TrafficControlDetail::MINHDWY_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[1].params.detail.minhdwy,2);

  EXPECT_EQ(traffic_mobility_msg_test[2].params.detail.choice,cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE);
  EXPECT_EQ(traffic_mobility_msg_test[2].params.detail.maxspeed,1.2);

}

}//traffic
