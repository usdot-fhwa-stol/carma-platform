
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#include <carma_v2x_msgs/msg/mobility_request.hpp>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_v2x_msgs/msg/mobility_header.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <carma_v2x_msgs/msg/trajectory.hpp>
#include <carma_v2x_msgs/msg/location_offset_ecef.hpp>
using namespace std;

class MobilityMessages{
    
    private:
        carma_v2x_msgs::msg::MobilityRequest req1;
        carma_v2x_msgs::msg::MobilityRequest req2;
        carma_v2x_msgs::msg::MobilityRequest req3;
        // UCLA add req4 to test added PlanType
        carma_v2x_msgs::msg::MobilityRequest req4;
        carma_v2x_msgs::msg::MobilityResponse res1;
        carma_v2x_msgs::msg::MobilityResponse res2;
        carma_v2x_msgs::msg::MobilityResponse res3;
        carma_v2x_msgs::msg::MobilityOperation op1;
        carma_v2x_msgs::msg::MobilityOperation op2;
        carma_v2x_msgs::msg::MobilityOperation op3;
    
    public:   
        MobilityMessages(){
            carma_v2x_msgs::msg::MobilityHeader header;
            carma_v2x_msgs::msg::PlanType plan;
            carma_v2x_msgs::msg::LocationECEF location;
            carma_v2x_msgs::msg::LocationOffsetECEF offset;
            carma_v2x_msgs::msg::Trajectory traject;
            vector<carma_v2x_msgs::msg::LocationOffsetECEF> offsets;
            //mobility request messages            

            //set up location
            location.ecef_x = 10000;
            location.ecef_y = 0;
            location.ecef_z = 33333;
            location.timestamp = 99999;
            
            
            //set up trajectory
            traject.location = location;
            offset.offset_x = 14;
            offset.offset_y = 200;
            offset.offset_z = 150;
            offsets.push_back(offset);
            traject.offsets = offsets;

            //set up plan
            plan.type = 1;
            
            //set up header
            header.sender_id = "sender_id1";
            header.sender_bsm_id = "sender_bsm_id1";
            header.recipient_id = "";
            header.plan_id = "plan_id1";
            header.timestamp = 55555555555555;
            
            //set up mobility request
            req1.m_header = header;
            req1.strategy = "CARMA/platooning";
            req1.plan_type = plan;
            req1.urgency = 1000;
            req1.location = location;
            req1.strategy_params = "param1 param2";
            req1.trajectory = traject;
            req1.expiration = 5555555;


            //set up location
            location.ecef_x = 40;
            location.ecef_y = 12312321;
            location.ecef_z = 3177;
            location.timestamp = 191934;
            
            
            //set up trajectory
            traject.location = location;
            offset.offset_x = 189;
            offset.offset_y = 2123124;
            offset.offset_z = 14;
            offsets.push_back(offset);
            traject.offsets = offsets;

            //set up plan
            plan.type = 2;
            
            //set up header
            header.sender_id = "11";
            header.sender_bsm_id = "sender_bsm_id5";
            header.recipient_id = "recipient_id5";
            header.plan_id = "plan_id5";
            header.timestamp = 555555555555;
            
            //set up mobility request
            req2.m_header = header;
            req2.strategy = "CARMA/platooning";
            req2.plan_type.type = carma_v2x_msgs::msg::PlanType::PLATOON_FOLLOWER_JOIN;
            req2.urgency = 1000;
            req2.location = location;
            req2.strategy_params = "param1 param2 param3 param4";
            req2.trajectory = traject;
            req2.expiration = 98124172;

            
            //set up location
            location.ecef_x = 1238123;
            location.ecef_y = 9875488;
            location.ecef_z = 3434534;
            location.timestamp = 88888888;
            
            
            //set up trajectory
            traject.location = location;
            offset.offset_x = 987893934;
            offset.offset_y = 5151;
            offset.offset_z = 12312;
            offsets.push_back(offset);
            traject.offsets = offsets;

            //set up plan
            plan.type = 3;
            
            //set up header
            header.sender_id = "sender_id6";
            header.sender_bsm_id = "sender_bsm_id6";
            header.recipient_id = "recipient_id6";
            header.plan_id = "plan_id6";
            header.timestamp = 98745745;
            
            //set up mobility request
            req3.m_header = header;
            req3.strategy = "CARMA/platooning3";
            req3.plan_type = plan;
            req3.urgency = 156;
            req3.location = location;
            req3.strategy_params = "param1 param2";
            req3.trajectory = traject;
            req3.expiration = 15132;
            
            // UCLA: set up req4 to test platoon_front_join

            //set up location
            location.ecef_x = 1223232;
            location.ecef_y = 2323488;
            location.ecef_z = 2323223;
            location.timestamp = 66666666;
            
            
            //set up trajectory
            traject.location = location;
            offset.offset_x = 232;
            offset.offset_y = 23232334;
            offset.offset_z = 23;
            offsets.push_back(offset);
            traject.offsets = offsets;

            //set up plan
            plan.type = 4;
            
            //set up header
            header.sender_id = "23";
            header.sender_bsm_id = "sender_bsm_id2";
            header.recipient_id = "recipient_id2";
            header.plan_id = "plan_id2";
            header.timestamp = 2222222222222;
            
            //set up mobility request
            req4.m_header = header;
            req4.strategy = "CARMA/platooning";
            req4.plan_type.type = carma_v2x_msgs::msg::PlanType::PLATOON_FRONT_JOIN;
            req4.urgency = 1300;
            req4.location = location;
            req4.strategy_params = "param1 param2 param3 param4";
            req4.trajectory = traject;
            req4.expiration = 9823232323;

            
            //mobility response messages


            //set up header
            header.sender_id = "sender_id2";
            header.sender_bsm_id = "sender_bsm_id2";
            header.recipient_id = "recipient_id1";
            header.plan_id = "plan_id2";
            header.timestamp = 66666666;

            //set up mobility response
            res1.m_header = header;
            res1.is_accepted = true;
            res1.urgency = 1000;

             //set up header
            header.sender_id = "sender_id9";
            header.sender_bsm_id = "sender_bsm_id9";
            header.recipient_id = "recipient_id9";
            header.plan_id = "plan_id9";
            header.timestamp = 1772327;

            //set up mobility response
            res2.m_header = header;
            res2.is_accepted = false;
            res2.urgency = 50;

             //set up header
            header.sender_id = "sender_id10";
            header.sender_bsm_id = "sender_bsm_id10";
            header.recipient_id = "recipient_id10";
            header.plan_id = "plan_id10";
            header.timestamp = 77777777;

            //set up mobility response
            res3.m_header = header;
            res3.is_accepted = true;
            res3.urgency = 700;




            //mobility operation messages

            //set up header
            header.sender_id = "sender_id3";
            header.sender_bsm_id = "sender_bsm_id3";
            header.recipient_id = "recipient_id3";
            header.plan_id = "plan_id3";
            header.timestamp = 44444444;

            op1.m_header = header;
            op1.strategy = "STATUS";
            op1.strategy_params = "CMDSPEED:1.0,DTD:1.0,SPEED:1.0";

            //set up header
            header.sender_id = "sender_id12";
            header.sender_bsm_id = "sender_bsm_id12";
            header.recipient_id = "recipient_id12";
            header.plan_id = "plan_id12";
            header.timestamp = 8989898;

            op2.m_header = header;
            op2.strategy = "strategy3";
            op2.strategy_params = "param1 param2 param3 param4 param5";


            //set up header
            header.sender_id = "sender_id23";
            header.sender_bsm_id = "sender_bsm_id23";
            header.recipient_id = "recipient_id23";
            header.plan_id = "plan_id23";
            header.timestamp = 181818;

            op3.m_header = header;
            op3.strategy = "strategy23";
            op3.strategy_params = "";
        
        }

        carma_v2x_msgs::msg::MobilityRequest getRequest1(){
            return req1;
        }

        carma_v2x_msgs::msg::MobilityRequest getRequest2(){
            return req2;
        }
        
        carma_v2x_msgs::msg::MobilityRequest getRequest3(){
            return req3;
        }

        
        carma_v2x_msgs::msg::MobilityResponse getResponse1(){
            return res1;
        }

        carma_v2x_msgs::msg::MobilityResponse getResponse2(){
            return res2;
        }

        carma_v2x_msgs::msg::MobilityResponse getResponse3(){
            return res3;
        }

        carma_v2x_msgs::msg::MobilityOperation getOperation1(){
            return op1;
        }

        carma_v2x_msgs::msg::MobilityOperation getOperation2(){
            return op2;
        }

        carma_v2x_msgs::msg::MobilityOperation getOperation3(){
            return op3;
        }

};
