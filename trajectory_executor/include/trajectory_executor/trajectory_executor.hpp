/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#ifndef __TRAJECTORY_EXECUTOR_HPP__
#define __TRAJECTORY_EXECUTOR_HPP__

#include <ros/ros.h>
#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <cav_msgs/TrajectoryPlan.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <carma_utils/CARMAUtils.h>
#include <ros/callback_queue.h>

namespace trajectory_executor {
    cav_msgs::TrajectoryPlan trimFirstPoint(const cav_msgs::TrajectoryPlan plan);

    class TrajectoryExecutor {
        public:
            TrajectoryExecutor(int traj_frequency);
            TrajectoryExecutor();
            bool init();
            void run();
        protected:
            std::map<std::string, std::string> queryControlPlugins();
            void onNewTrajectoryPlan(cav_msgs::TrajectoryPlan msg);
            void onTrajEmitTick(const ros::TimerEvent& te);
        private:
            std::unique_ptr<ros::CARMANodeHandle> _default_nh;
            std::unique_ptr<ros::CARMANodeHandle> _timer_nh;
            std::unique_ptr<ros::CARMANodeHandle> _msg_nh;
            ros::CallbackQueue _timer_callbacks;
            ros::CallbackQueue _msg_callbacks;
            ros::Subscriber _plan_sub;
            std::map<std::string, ros::Publisher> _traj_publisher_map;
            std::unique_ptr<cav_msgs::TrajectoryPlan> _cur_traj;
            std::mutex _cur_traj_mutex;
            int _min_traj_publish_tickrate_hz;
            ros::Timer _timer;
            int _timesteps_since_last_traj;
            int _default_spin_rate;
    };
}

#endif 
