/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <functional>
#include <cstdlib>


#include <visualization_msgs/MarkerArray.h>
#include <cav_msgs/ExternalObjectList.h>
#include <std_msgs/ColorRGBA.h>

std::vector< std_msgs::ColorRGBA > colors;
ros::Publisher pub_box;

void populateColors()
{
    std_msgs::ColorRGBA c;
    c.a = .8;

    c.r = .75; c.g = .75; c.b = .75;
    colors.push_back(c);

    c.r = 1; c.g = 1; c.b = 1;
    colors.push_back(c);

    c.r = .5; c.g = 0; c.b = 0;
    colors.push_back(c);

    c.r = 1; c.g = 0; c.b = 0;
    colors.push_back(c);

    c.r = .5; c.g = .5; c.b = 0;
    colors.push_back(c);

    c.r = 1; c.g = 1; c.b = 0;
    colors.push_back(c);

    c.r = 0; c.g = .5; c.b = .0;
    colors.push_back(c);

    c.r = 0; c.g = 1; c.b = 0;
    colors.push_back(c);

    c.r = 0; c.g = .5; c.b = .5;
    colors.push_back(c);

    c.r = 0, c.g = 1; c.b = 1;
    colors.push_back(c);

    c.r = 0; c.g = 0; c.b = .5;
    colors.push_back(c);

    c.r = 0; c.g = 0; c.b = 1;
    colors.push_back(c);

    c.r = .5; c.g = 0; c.b = .5;
    colors.push_back(c);

    c.r = 1; c.g = 0; c.b = 1;
    colors.push_back(c);
}

void solCb (const cav_msgs::ExternalObjectList::ConstPtr& sol_msg, int color)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;

    m.header = sol_msg->header;

    m.action = visualization_msgs::Marker::ADD;
    m.scale.x=1;
    m.scale.y=1;
    m.scale.z=1;

    m.lifetime = ros::Duration(0);
    m.id = 1;

    for(auto it = sol_msg->objects.begin(); it != sol_msg->objects.end(); ++it)
    {
        m.type = visualization_msgs::Marker::CUBE;
        m.ns = "bounding_boxes";

        m.header = it->header;

        m.scale.x = 2.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;

        m.id = it->id;

        if(color < 0)
        {
            m.color = colors[m.id % (colors.size() - 1)];
        } 
        else 
        {
            m.color = colors[color % (colors.size() - 1)];
        }
        m.points.resize(0);

        m.pose.position.x = it->pose.pose.position.x;
        m.pose.position.y = it->pose.pose.position.y;
        m.pose.position.z = it->pose.pose.position.z;

        m.pose.orientation.w = it->pose.pose.orientation.w;
        m.pose.orientation.x = it->pose.pose.orientation.x;
        m.pose.orientation.y = it->pose.pose.orientation.y;
        m.pose.orientation.z = it->pose.pose.orientation.z;

        ma.markers.push_back(m);

        // Make an arrow in the center of the cubic marker that represents the velocity direction.
        m.ns = "arrows";
        m.type = visualization_msgs::Marker::ARROW;

        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m.scale.z = 1;

        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 0;

        m.pose.orientation.w = 1;
        m.pose.orientation.x = 0;
        m.pose.orientation.y = 0;
        m.pose.orientation.z = 0;

        geometry_msgs::Point s, e;
        s.x = it->pose.pose.position.x;
        s.y = it->pose.pose.position.y;
        s.z = it->pose.pose.position.z;

        e.x = it->pose.pose.position.x + it->velocity.twist.linear.x / 3;
        e.y = it->pose.pose.position.y + it->velocity.twist.linear.y / 3;
        e.z = it->pose.pose.position.z;
        //e.z = it->pose.position.z + it->velocity.linear.z;

        m.points.push_back(s);
        m.points.push_back(e);

        ma.markers.push_back(m);

        // Make a text box that prints the velocity and the object ID number.
        m.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.ns = "velocity_text";

        m.pose.position.x = it->pose.pose.position.x - 0.5;
        m.pose.position.y = it->pose.pose.position.y;
        m.pose.position.z = it->pose.pose.position.z;
        m.scale.x = 1;
        m.scale.y = 1;
        m.scale.z = 1;

        double velocity = sqrt((it->velocity.twist.linear.x * it->velocity.twist.linear.x) + (it->velocity.twist.linear.y * it->velocity.twist.linear.y));

        m.text = std::to_string(velocity);

        ma.markers.push_back(m);

        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.ns = "object_id";

        m.pose.position.x = it->pose.pose.position.x - 1.5;
        m.pose.position.y = it->pose.pose.position.y;
        m.pose.position.z = it->pose.pose.position.z;
        m.scale.x = 1;
        m.scale.y = 1;
        m.scale.z = 1;

        //m.text = std::to_string(it->velocity_confidence);
        m.text = std::to_string(it->id);

        ma.markers.push_back(m);

        // Make header
        m.type=visualization_msgs::Marker::TRIANGLE_LIST;
        m.ns = "heading";
        m.colors.clear();
        m.points.resize(0);
        m.scale.x = 1;
        m.scale.y = 1;
        m.scale.z = 1;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.color.a = 1.0;

        m.pose = it->pose.pose;

        double distance_to_corner = sqrt(2.0 * 2.0 / 4.0 + 1.0 * 1.0 / 4.0);
        double angle_to_corner = std::acos(2.0 / 2.0 / distance_to_corner);

        geometry_msgs::Point p, tip;
        tip.x = 1.5 * distance_to_corner;
        tip.y = 0;
        tip.z = 0.0;
        m.points.push_back(tip);

        p.x = distance_to_corner * std::cos(angle_to_corner);
        p.y = 0.0 - distance_to_corner * std::sin(angle_to_corner);
        p.z = 0.0 - 1.0 / 2.0;
        m.points.push_back(p);
        p.y *= (-1.0);
        m.points.push_back(p);
        
        m.points.push_back(p);
        p.z *= (-1.0);
        m.points.push_back(p);
        m.points.push_back(tip);
        
        m.points.push_back(p);
        p.y *= (-1.0);
        m.points.push_back(p);
        m.points.push_back(tip);
        
        m.points.push_back(p);
        p.z *= (-1.0);
        m.points.push_back(p);
        m.points.push_back(tip);

        std_msgs::ColorRGBA c;
        c.r = 1.0;
        c.g = 1.0;
        c.b = 1.0;
        c.a = 1.0;

        for (unsigned int i = 0; i < 12; i++)
        {
            m.colors.push_back(c);
        }

        ma.markers.push_back(m);
    }

    m.action = visualization_msgs::Marker::DELETEALL;

    ma.markers.insert(ma.markers.begin(), m);

    pub_box.publish(ma);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "marker_translate");
    ros::NodeHandle nh;

    populateColors();

    pub_box = nh.advertise<visualization_msgs::MarkerArray> ("/objects_markers", 1);

    // Default to set marker by object_id
    int test;
    if(argc < 2)
    {
        test = -1;
    } 
    else 
    {
        test = std::atol(argv[1]);
    }

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<cav_msgs::ExternalObjectList>("/objects", 1, std::bind(solCb, std::placeholders::_1, test));

    // Spin
    ros::spin ();
}
