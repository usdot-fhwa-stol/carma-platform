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

#include <cav_srvs/GetDriversWithCapabilities.h>

#include <ros/ros.h>

/**
 * @brief Just a test application to use with the sensor_fusion_node
 *
 * This application will respond to the sensor_fusion_node request for position data.
 * The application assumes the pinpoint node is running and is providing the position services
 * that are requested.
 *
 * This by no means shows the complete interaction with an interface manager. *
 */

bool getDriversWithCapabilities(cav_srvs::GetDriversWithCapabilitiesRequest& req,
                                cav_srvs::GetDriversWithCapabilitiesResponse& res)
{
    if(req.category != cav_srvs::GetDriversWithCapabilitiesRequest::POSITION)
    {
        res.driver_names.clear();
        return true;
    }

    res.driver_names.push_back("/pinpoint");

}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "interface_mgr");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("get_drivers_with_capabilities", getDriversWithCapabilities);

    ros::spin();
    return 0;
}