#include "ros/ros.h"
#include <sstream>
#include <cav_msgs/SpeedAccel.h>

void new_guidance_commands_Callback(const cav_msgs::SpeedAccel& msg){
    ROS_INFO("I heard: [%s]", msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "new_guidance_commands");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/test", 1000, chatterCallback);
    ros::spin();

    return 0;
}



    // ros::init(argc, argv, "talker");
    // ros::NodeHandle n;

    // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    // ros::Rate loop_rate(10);

    // int count = 0;
    // while (ros::ok()){
    //     std_msgs::String msg;
    //     std::stringstream ss;
    //     ss << "hello world " << count;
    //     msg.data = ss.str();

    //     ROS_INFO("%s", msg.data.c_str());
    //     chatter_pub.publish(msg);

    //     ros::spinOnce();
    //     loop_rate.sleep();

    //     ++count;
    // }

    // return 0;
