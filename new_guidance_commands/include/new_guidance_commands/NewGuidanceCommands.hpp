// ROS
#include <ros/ros.h>
#include <sstream>
// CAV
#include <cav_msgs/SpeedAccel.h>

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

    private:
        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool readParameters();

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void topicCallback(const cav_msgs::SpeedAccel& msg);

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;

        //! ROS topic subscriber.
        ros::Subscriber subscriber_;

        //! ROS topic name to subscribe to.
        std::string subscriberTopic_;

};

}  // namespace new_guidance_commands