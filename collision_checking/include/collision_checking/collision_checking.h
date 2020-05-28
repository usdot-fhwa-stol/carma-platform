#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/VehicleSize.h>
#include <cav_msgs/TrajectoryPlan.h>

#include <ros/ros.h>
#include <boost/geometry.hpp>

namespace collision_checking {
    /*!
    * Main class for the node to handle the ROS interfacing.
    */

    class CollisionChecking {
        public:
            /*!
            * Constructor.
            * @param nodeHandle the ROS node handle.
            */
            CollisionChecking(ros::NodeHandle& nodeHandle);

            /*!
            * Destructor.
            */
            virtual ~CollisionChecking();
            

        private:
            /*!
            * Reads and verifies the ROS parameters.
            * @return true if successful.
            */
            bool readParameters();

            //! ROS node handle.
            ros::NodeHandle& nodeHandle_;


    };
}