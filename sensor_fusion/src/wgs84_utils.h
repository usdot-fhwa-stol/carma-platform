#pragma once
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
#include <eigen3/Eigen/Geometry>

namespace wgs84_utils
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double EARTH_RADIUS_METERS = 6378137.0;
    constexpr double DEG2RAD = pi/180.0;
    constexpr double RAD2DEG = 180.0/pi;

    struct wgs84_coordinate
    {
        double lat, lon, heading, elevation;
    };

    double calcMetersPerRadLat(const wgs84_coordinate& tie_point);

    double calcMetersPerRadLon(const wgs84_coordinate& tie_point);

    void convertToOdom(const wgs84_coordinate& src,
                       const wgs84_coordinate& wgs84_ref,
                       const Eigen::Vector3d& odom_pose_ref,
                       const Eigen::Quaternion<double>& odom_rot_ref,
                       const Eigen::Transform<double, 3, Eigen::Affine>& ned_odom_tf,
                       Eigen::Vector3d& out_pose,
                       Eigen::Quaternion<double>& out_rot
    );
}
