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
        
        void publisher();

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
        void speedAccel_SubscriberCallback(const cav_msgs::SpeedAccel& msg);



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
        void lateralControl_SubscriberCallback(const cav_msgs::LateralControl& msg);



        //! ROS  enable_robotic subscriber.
        ros::Subscriber enable_roboticsubscriber_;
        std::string enable_robotic_subscriberTopic_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void enable_robotic_SubscriberCallback(const cav_msgs::RobotEnabled& msg);


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


        ros::Publisher  enable_robotic_publisher_;
        std::string  enable_robotic_publisherTopic_;

        /*!
        * ROS  enable_robotic_publisher method.
        * @param message the received message.
        */
        void enable_robotic_Publisher();


        cav_msgs::SpeedAccel SpeedAccel_msg;
        std_msgs::Float32::ConstPtr WrenchEffort_msg;
        cav_msgs::LateralControl LateralControl_msg;
        cav_msgs::RobotEnabled RobotEnabled_msg;
        
};

}  // namespace new_guidance_commands