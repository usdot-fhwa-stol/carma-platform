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
#include <tf2/LinearMath/Transform.h>

namespace wgs84_utils
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double DEG2RAD = pi/180.0;
    constexpr double RAD2DEG = 180.0/pi;
    // Earth Geometry Constants
    constexpr double Rea = 6378137.0; // Semi-major axis radius meters
    constexpr double Rea_sqr = Rea*Rea;
    constexpr double f = 1.0 / 298.257223563; //The flattening factor
    constexpr double Reb = Rea * (1.0 - f); // //The semi-minor axis = 6356752.0
    constexpr double Reb_sqr = Reb*Reb;
    constexpr double e = 0.08181919084262149; // The first eccentricity (hard coded as optimization) calculated as Math.sqrt(Rea*Rea - Reb*Reb) / Rea;
    constexpr double e_sqr = e*e;
    constexpr double e_p = 0.08209443794969568; // e prime (hard coded as optimization) calculated as Math.sqrt((Rea_sqr - Reb_sqr) / Reb_sqr);

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

    tf2::Vector3 geodesic_to_ecef(const wgs84_coordinate &loc, tf2::Transform ecef_in_ned);

    wgs84_coordinate ecef_to_geodesic(const tf2::Vector3& point);

    tf2::Transform ecef_to_ned_from_loc(wgs84_coordinate loc);

}
