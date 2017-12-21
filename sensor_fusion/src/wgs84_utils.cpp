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
#include "wgs84_utils.h"

#include <cmath>
/**
 * @brief gets the meters per radian of latitude at a global coordinate
 * @param tie_point
 * @return
 */
double wgs84_utils::calcMetersPerRadLat(const wgs84_utils::wgs84_coordinate &tie_point) {
    double a = EARTH_RADIUS_METERS;
    double e_sq = 0.00669438;
    double s_sq = pow(sin(tie_point.lat), 2);

    double R_m = a*(1.0-e_sq)/pow(sqrt(1.0-e_sq*s_sq), 3);

    double meters_per_rad_lat = R_m + tie_point.elevation;

    return meters_per_rad_lat;
}


/**
 * @brief gets the meters per radian of longitude at a global coordinate
 * @param tie_point
 * @return
 */
double wgs84_utils::calcMetersPerRadLon(const wgs84_utils::wgs84_coordinate &tie_point) {
    double a = EARTH_RADIUS_METERS;
    double e_sq = 0.00669438;
    double s_sq = pow(sin(tie_point.lat), 2);

    double R_n = a/(sqrt(1.0-e_sq*s_sq));

    double meters_per_rad_lon = (R_n + tie_point.elevation) * cos(tie_point.lat);

    return meters_per_rad_lon;
}


/**
 * @brief Converts a global coordinate in NED to a local coordinate frame in FLU based on a reference global/local pair
 * @param src - the global coordinate to be converted
 * @param wgs84_ref - the global reference
 * @param odom_pose_ref - the local reference in FLU
 * @param odom_rot_ref - the local rotation ref in FLU
 * @param ned_odom_tf - the ned odometry transform
 * @param out_pose
 * @param out_rot
 */
void wgs84_utils::convertToOdom(const wgs84_utils::wgs84_coordinate &src,
                                const wgs84_utils::wgs84_coordinate &wgs84_ref,
                                const Eigen::Vector3d &odom_pose_ref,
                                const Eigen::Quaternion<double>& odom_rot_ref,
                                const Eigen::Transform<double,3,Eigen::Affine>& ned_odom_tf,
                                Eigen::Vector3d &out_pose,
                                Eigen::Quaternion<double> &out_rot)
{
    double delta_lat = src.lat - wgs84_ref.lat;
    double delta_lon = src.lon - wgs84_ref.lon;
    double delta_elev = src.elevation - wgs84_ref.elevation;
    double delta_heading = src.heading - wgs84_ref.heading;

    //calculate translation
    Eigen::Vector3d offset;
    offset[0] = delta_lat*calcMetersPerRadLat(wgs84_ref);
    offset[1] = delta_lon*calcMetersPerRadLon(wgs84_ref);
    offset[2] = delta_elev;

    offset = ned_odom_tf*offset;

    out_pose = odom_pose_ref + offset;

    //Calculate orientation

    Eigen::Quaternion<double> q_offset;
    q_offset = Eigen::AngleAxisd(delta_heading*DEG2RAD,Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> neu_odom_rot(ned_odom_tf.rotation());

    out_rot =  neu_odom_rot*q_offset*odom_rot_ref;
}
