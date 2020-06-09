#include <std_srvs/Trigger.h>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <boost/assign/std/vector.hpp>

#include <iostream>
#include <fstream>

namespace platooning_tactical_plugin {

    /*!
    * Main class for the PlatooningTacticalPlugin interfacing.
    */

    class PlatooningTacticalPlugin {
        public:
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            PlatooningTacticalPlugin(ros::NodeHandle& nodeHandle);


            // final waypoint subscriber
            // service call plan trajectory request respose

            // waypoint to trajectory convertor

            /*!
            * Destructor.
            */
            virtual ~PlatooningTacticalPlugin();

        private:

            //! ROS node handle.
            ros::NodeHandle& nodeHandle_;

            /*!
            * Reads and verifies the ROS parameters.
            * @return true if successful.
            */
            bool readParameters();

            //! ROS speedAccel subscriber.
            ros::Subscriber waypoint_subscriber_;

            void waypoint_subscriber_callback(const autoware_msgs::Lane::ConstPtr& msg);

            bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    };
}