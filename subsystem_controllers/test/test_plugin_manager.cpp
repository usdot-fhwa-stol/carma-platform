/*
 * Copyright (C) 2021 LEIDOS.
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
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <thread>
#include <chrono>

#include <ros2_lifecycle_manager/lifecycle_manager_interface.hpp>

#include "subsystem_controllers/guidance_controller/plugin_manager.h"
#include <lifecycle_msgs/msg/state.hpp>

using std_msec = std::chrono::milliseconds;
using std_nanosec = std::chrono::nanoseconds;

namespace subsystem_controllers
{

    uint8_t UNKOWN = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    uint8_t UNCONFIGURED = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    uint8_t INACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    uint8_t ACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    uint8_t FINALIZED = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;


    class MockLifecycleManager : public ros2_lifecycle_manager::LifecycleManagerInterface
    {
        public:

        std::unordered_map<std::string, uint8_t> node_states;

        virtual ~MockLifecycleManager(){};

        void set_managed_nodes(const std::vector<std::string> &nodes) 
        { 
            for (auto n : nodes) 
            {
                node_states.insert({n, UNKOWN});
            }
        }

        void add_managed_node(const std::string& node){ node_states.insert({node, UNKOWN}); }

        std::vector<std::string> get_managed_nodes() 
        {
            std::vector<std::string> keys;
            for(auto const& n: node_states)
                keys.push_back(n.first);

            return keys;
        }

        uint8_t get_managed_node_state(const std::string &node) { return node_states.at(node); };


        uint8_t transition_node_to_state(const uint8_t state, const std::string& node, const std_nanosec &, const std_nanosec &) 
        {
            node_states[node] = state;
            return state;
        }

        std::vector<std::string> simple_transition(uint8_t state, std::vector<std::string> nodes = {})
        {
            if (nodes.empty())
            {
                for (auto n : node_states)
                {
                    transition_node_to_state(state, n.first, std::chrono::milliseconds(10), std::chrono::milliseconds(10));
                }

                return {};
            }

            for (auto n : nodes)
            {
                transition_node_to_state(state, n, std::chrono::milliseconds(10), std::chrono::milliseconds(10));
            }

            return {};
        }

        std::vector<std::string> configure(const std_nanosec &, const std_nanosec &, bool, std::vector<std::string> nodes) 
        {
            return simple_transition(INACTIVE, nodes);
        };

        std::vector<std::string> cleanup(const std_nanosec &, const std_nanosec &, bool, std::vector<std::string> nodes)
        {
            return simple_transition(UNCONFIGURED, nodes);
        }

        std::vector<std::string> activate(const std_nanosec &, const std_nanosec &, bool, std::vector<std::string> nodes)
        {
            return simple_transition(ACTIVE, nodes);
        }

        std::vector<std::string> deactivate(const std_nanosec &, const std_nanosec &, bool, std::vector<std::string> nodes)
        {
            return simple_transition(INACTIVE, nodes);
        }

        std::vector<std::string> shutdown(const std_nanosec &, const std_nanosec &, bool, std::vector<std::string> nodes)
        {
            return simple_transition(FINALIZED, nodes);
        }

    };

    TEST(plugin_manager_test, transitions_test)
    {
        std::vector<std::string> req_plugins = {"plg_1"};
        std::vector<std::string> auto_actv_plugins = {"plg_2"};
        std::vector<std::string> ros2_plugins = {"plg_2", "plg_1"};

        auto mlm = std::make_shared<MockLifecycleManager>();

        uint8_t curr_state = UNCONFIGURED;

        PluginManager pm(
            req_plugins, 
            auto_actv_plugins, 
            ros2_plugins,
            mlm, 
            [&curr_state](){ return curr_state; },
            [](auto node, auto) -> std::map<std::string, std::vector<std::string, std::allocator<std::string>>> { 
                if (node=="plg_1")
                {
                    return { {"plg_1/change_state", { "lifecycle_msgs/srv/ChangeState" } },
                             {"plg_1/get_state", { "lifecycle_msgs/srv/GetState" } } };
                } else if (node == "plg_2")
                {
                    return { {"plg_2/change_state", {"lifecycle_msgs/srv/ChangeState"}},
                             {"plg_2/get_state", {"lifecycle_msgs/srv/GetState"}} };
                }
                else if (node == "plg_3")
                {
                    return { {"plg_3/change_state", {"lifecycle_msgs/srv/ChangeState"}},
                             {"plg_3/get_state", {"lifecycle_msgs/srv/GetState"}} };
                } else
                {
                    return {};
                }
            },
            std_msec(100), std_msec(100)
        );

        ///// CONFIGURE /////
        curr_state=INACTIVE;
        ASSERT_TRUE(pm.configure());

        ASSERT_EQ(mlm->get_managed_node_state("plg_1"), INACTIVE);
        ASSERT_EQ(mlm->get_managed_node_state("plg_2"), INACTIVE);

        SrvHeader hdr;

        auto req = std::make_shared<carma_planning_msgs::srv::PluginList::Request>();
        auto res = std::make_shared<carma_planning_msgs::srv::PluginList::Response>();

        pm.get_registered_plugins(hdr, req, res);

        ASSERT_EQ(res->plugins.size(), 2ul);

        bool has_p1 = false;
        bool has_p2 = false;

        for(auto p : res->plugins)
        {
            if (p.name == "plg_1")
                has_p1 = true;
            if (p.name == "plg_2")
                has_p2 = true;
        }

        ASSERT_TRUE(has_p1);
        ASSERT_TRUE(has_p2);

        res = std::make_shared<carma_planning_msgs::srv::PluginList::Response>();
        pm.get_active_plugins(hdr, req, res);

        ASSERT_TRUE(res->plugins.empty());

        //// ACTIVATE ////
        ASSERT_TRUE(pm.activate());
        curr_state=ACTIVE;

        ASSERT_EQ(mlm->get_managed_node_state("plg_1"), ACTIVE);
        ASSERT_EQ(mlm->get_managed_node_state("plg_2"), ACTIVE);

        res = std::make_shared<carma_planning_msgs::srv::PluginList::Response>();
        pm.get_active_plugins(hdr, req, res);

        has_p1 = false;
        has_p2 = false;

        for(auto p : res->plugins)
        {
            if (p.name == "plg_1")
                has_p1 = true;
            if (p.name == "plg_2")
                has_p2 = true;
        }

        ASSERT_TRUE(has_p1);
        ASSERT_TRUE(has_p2);

        auto api_req = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Request>();
        auto api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();

        // We have not yet send plugin discovery messages so the type and capabilities of the activated plugins is still unknown
        api_req->capability = "strategic_plan/plan_maneuvers";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_strategic_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_TRUE(api_res->plan_service.empty());
        
        api_req->capability = "tactical_plan/plan_trajectory";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_tactical_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_TRUE(api_res->plan_service.empty()); 

        api_req->capability = "control/trajectory_control";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_control_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_TRUE(api_res->plan_service.empty());

        // Add discovery messages for the two current plugins

        auto plugin_1_stat = std::make_unique<carma_planning_msgs::msg::Plugin>();
        plugin_1_stat->name = "plg_1";
        plugin_1_stat->version_id = "v1";
        plugin_1_stat->type = carma_planning_msgs::msg::Plugin::STRATEGIC;
        plugin_1_stat->available = true;
        plugin_1_stat->activated = true;
        plugin_1_stat->capability = "strategic_plan/plan_maneuvers";
        pm.update_plugin_status(std::move(plugin_1_stat));

        auto plugin_2_stat = std::make_unique<carma_planning_msgs::msg::Plugin>();
        plugin_2_stat->name = "plg_2";
        plugin_2_stat->version_id = "v1";
        plugin_2_stat->type = carma_planning_msgs::msg::Plugin::TACTICAL;
        plugin_2_stat->available = true;
        plugin_2_stat->activated = true;
        plugin_2_stat->capability = "tactical_plan/plan_trajectory";
        pm.update_plugin_status(std::move(plugin_2_stat));

        api_req->capability = "strategic_plan/plan_maneuvers";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_strategic_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_EQ(api_res->plan_service.size(), 1ul);

        ASSERT_TRUE(api_res->plan_service[0] == "plg_1/plan_maneuvers");
        
        api_req->capability = "tactical_plan/plan_trajectory";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_tactical_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_EQ(api_res->plan_service.size(), 1ul);

        ASSERT_TRUE(api_res->plan_service[0] == "plg_2/plan_trajectory");

        api_req->capability = "control/trajectory_control";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_control_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_TRUE(api_res->plan_service.empty());

        // Add a controller plugin
        auto plugin_3_stat = std::make_unique<carma_planning_msgs::msg::Plugin>();
        plugin_3_stat->name = "plg_3";
        plugin_3_stat->version_id = "v1";
        plugin_3_stat->type = carma_planning_msgs::msg::Plugin::CONTROL;
        plugin_3_stat->available = true;
        plugin_3_stat->activated = true;
        plugin_3_stat->capability = "control/trajectory_control";
        pm.update_plugin_status(std::move(plugin_3_stat));

        api_req->capability = "strategic_plan/plan_maneuvers";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_strategic_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_EQ(api_res->plan_service.size(), 1ul);

        ASSERT_TRUE(api_res->plan_service[0] == "plg_1/plan_maneuvers");
        
        api_req->capability = "tactical_plan/plan_trajectory";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_tactical_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_EQ(api_res->plan_service.size(), 1ul);

        ASSERT_TRUE(api_res->plan_service[0] == "plg_2/plan_trajectory");

        // ACTIVATE plugin 3
        auto activate_req = std::make_shared<carma_planning_msgs::srv::PluginActivation::Request>();
        auto activate_res = std::make_shared<carma_planning_msgs::srv::PluginActivation::Response>();
        activate_req->plugin_name = "plg_3"; 
        activate_req->plugin_version = "v1";
        activate_req->activated = true;
        pm.activate_plugin(hdr, activate_req, activate_res);

        ASSERT_TRUE(activate_res->newstate);

        api_req->capability = "control/trajectory_control";
        api_res = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Response>();
        pm.get_control_plugins_by_capability(hdr, api_req, api_res);

        ASSERT_EQ(api_res->plan_service.size(), 1ul);

        ASSERT_TRUE(api_res->plan_service[0] == "plg_3/plan_trajectory");

        res = std::make_shared<carma_planning_msgs::srv::PluginList::Response>();
        pm.get_active_plugins(hdr, req, res);

        has_p1 = false;
        has_p2 = false;
        bool has_p3 = false;

        for(auto p : res->plugins)
        {
            if (p.name == "plg_1")
                has_p1 = true;
            if (p.name == "plg_2")
                has_p2 = true;
            if (p.name == "plg_3")
                has_p3 = true;
        }

        ASSERT_TRUE(has_p1);
        ASSERT_TRUE(has_p2);
        ASSERT_TRUE(has_p3);

        // Deactivate plugin 3
        activate_res = std::make_shared<carma_planning_msgs::srv::PluginActivation::Response>();
        activate_req->activated = false;
        pm.activate_plugin(hdr, activate_req, activate_res);

        ASSERT_FALSE(activate_res->newstate);

        res = std::make_shared<carma_planning_msgs::srv::PluginList::Response>();
        pm.get_active_plugins(hdr, req, res);

        has_p1 = false;
        has_p2 = false;
        has_p3 = false;

        for(auto p : res->plugins)
        {
            if (p.name == "plg_1")
                has_p1 = true;
            if (p.name == "plg_2")
                has_p2 = true;
            if (p.name == "plg_3")
                has_p3 = true;
        }

        ASSERT_TRUE(has_p1);
        ASSERT_TRUE(has_p2);
        ASSERT_FALSE(has_p3);
        
    }

}
