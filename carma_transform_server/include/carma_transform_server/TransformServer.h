#pragma once

/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <cav_srvs/GetTransform.h>

/**
 * \class TransformServer
 * \brief ROS Node which maintains a tf2 transform tree which can be accessed by other nodes.
 *
 * The get_transform service can be used to obtain coordinate transformations between two frames.
 * Only transforms published on the /tf or /tf_static topics are recorded by this node.
 */
class TransformServer
{
  private:
    // Ros node handle
    ros::NodeHandle node_;

    // Buffer which holds the tree of transforms
    tf2_ros::Buffer tfBuffer_;
    // tf2 listeners. Subscribes to the /tf and /tf_static topics
    tf2_ros::TransformListener tfListener_;

    // get_transform service server
    ros::ServiceServer get_transform_service_;

  public:
    /**
     * \brief Constructor
     *
     * \param[in] argc command line argument count
     * \param[in] argv command line arguments
     */
    TransformServer(int argc, char** argv);

    /**
     * \brief Starts the application
     *
     * \return 0 on exit with no errors
     */
     int run();

  private:
    /**
      * \brief Callback function to the get_transform ros service.
      * \details Calculates the transform between the requested frames using a tf2_ros::Buffer
      *
      * \param[in] req - The service request
      * \param[out] res - The service response
      *
      * \return True when the callback completes
      */
    bool get_transform_cb(cav_srvs::GetTransform::Request  &req, cav_srvs::GetTransform::Response &res);
};