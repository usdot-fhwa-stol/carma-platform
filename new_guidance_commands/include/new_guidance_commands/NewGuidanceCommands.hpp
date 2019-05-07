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

#include <mutex>
// ROS
#include <ros/ros.h>
#include <sstream>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
// CAV
#include <cav_msgs/SpeedAccel.h>
#include <cav_msgs/LateralControl.h>
#include <cav_msgs/RobotEnabled.h>

namespace new_guidance_commands {

/*!
 * Main class for the node to handle the ROS interfacing.
 */

class NewGuidanceCommands {
    public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        NewGuidanceCommands(ros::NodeHandle& nodeHandle);

        /*!
        * Destructor.
        */
        virtual ~NewGuidanceCommands();
        
        // Calls speedAccel_Publisher and wrenchEffort_Publisher and lateralControl_Publisher
        void publisher();

        // runs publish at a desired frequency
        int rate;

    private:
        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool readParameters();

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;

        //! ROS speedAccel subscriber.
        ros::Subscriber speedAccelsubscriber_;
        std::string speedAccel_subscriberTopic_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void speedAccel_SubscriberCallback(const cav_msgs::SpeedAccel::ConstPtr& msg);



        //! ROS wrenchEffort subscriber.
        ros::Subscriber wrenchEffortsubscriber_;
        std::string wrenchEffort_subscriberTopic_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void wrenchEffort_SubscriberCallback(const std_msgs::Float32::ConstPtr& msg);



        //! ROS lateralControl subscriber.
        ros::Subscriber lateralControlsubscriber_;
        std::string lateralControl_subscriberTopic_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void lateralControl_SubscriberCallback(const cav_msgs::LateralControl::ConstPtr& msg);


        //! ROS topic publishers.
        ros::Publisher speedAccel_publisher_;
        std::string speedAccel_publisherTopic_;

        /*!
        * ROS speedAccel publisher method.
        * @param message the received message.
        */
        void speedAccel_Publisher();


        ros::Publisher wrenchEffort_publisher_;
        std::string wrenchEffort_publisherTopic_;

        /*!
        * ROS wrenchEffort publisher method.
        * @param message the received message.
        */
        void wrenchEffort_Publisher();

        
        ros::Publisher lateralControl_publisher_;
        std::string lateralControl_publisherTopic_;

        /*!
        * ROS lateralControl_publisher method.
        * @param message the received message.
        */
        void lateralControl_Publisher();


        // Messages used to transfer data from subscribers to publishers
        cav_msgs::SpeedAccel::ConstPtr SpeedAccel_msg;
        std_msgs::Float32::ConstPtr WrenchEffort_msg;
        cav_msgs::LateralControl::ConstPtr LateralControl_msg;

        // Locks used to make the data transfer from subscribers to publishers thread safe
        std::mutex SpeedAccel_msg_mutex;
        std::mutex WrenchEffort_msg_mutex;
        std::mutex LateralControl_msg_mutex;

        // Time used to check the timestamp for the last time subscribers reciaved message
        double SpeedAccelTimeTracker;
        double WrenchEffortTimeTracker;
        double LateralControlTimeTracker;

        ros::Duration TimeoutThresh;
        double timeout;

        void InitTimeTracker();

};

}  // namespace new_guidance_commands