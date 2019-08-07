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
#include "wgs84_utils/wgs84_utils.h"

#include <cmath>
/**
 * @brief gets the meters per radian of latitude at a global coordinate
 * @param tie_point
 * @return
 */
double wgs84_utils::calcMetersPerRadLat(const wgs84_utils::wgs84_coordinate &tie_point)
{
    double s_sq = pow(sin(tie_point.lat * DEG2RAD), 2);

    double R_m = Rea * (1.0 - e_sqr) / pow(sqrt(1.0 - e_sqr * s_sq), 3);

    double meters_per_rad_lat = R_m + tie_point.elevation;

    return meters_per_rad_lat;
}

/**
 * @brief gets the meters per radian of longitude at a global coordinate
 * @param tie_point
 * @return
 */
double wgs84_utils::calcMetersPerRadLon(const wgs84_utils::wgs84_coordinate &tie_point)
{
    double s_sq = pow(sin(tie_point.lat * DEG2RAD), 2);

    double R_n = Rea / (sqrt(1.0 - e_sqr * s_sq));

    double meters_per_rad_lon = (R_n + tie_point.elevation) * cos(tie_point.lat * DEG2RAD);

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
                                const Eigen::Transform<double, 3, Eigen::Affine>& ned_odom_tf,
                                Eigen::Vector3d &out_pose,
                                Eigen::Quaternion<double> &out_rot)
{
    double delta_lat = src.lat - wgs84_ref.lat;
    double delta_lon = src.lon - wgs84_ref.lon;
    double delta_elev = src.elevation - wgs84_ref.elevation;
    double delta_heading = src.heading - wgs84_ref.heading;

    // Calculate translation
    Eigen::Vector3d ned_offset;
    ned_offset[0] = delta_lat * DEG2RAD * calcMetersPerRadLat(wgs84_ref);
    ned_offset[1] = delta_lon * DEG2RAD * calcMetersPerRadLon(wgs84_ref);
    ned_offset[2] = -delta_elev;

    auto ned_pose = ned_odom_tf.inverse() * odom_pose_ref;
    ned_pose += ned_offset;
    out_pose = ned_odom_tf*ned_pose;

    // Calculate orientation

    Eigen::Quaternion<double> q_offset;
    q_offset = Eigen::AngleAxisd(delta_heading * DEG2RAD, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> neu_odom_rot(ned_odom_tf.rotation());

    out_rot =  neu_odom_rot * q_offset * odom_rot_ref;
}

/**
 * Converts a given WSG-84 geodesic location into a 3d cartesian point
 * The returned 3d point is defined relative to a frame which has a transform with the ECEF frame.
 * @param location The geodesic location to convert must be in radians
 * @param frame2ecefTransform A transform which defines the location of the ECEF frame relative to the 3d point's frame of origin
 * @return The calculated 3d point in the ECEF frame. 
 */
tf2::Vector3 wgs84_utils::geodesic_to_ecef(const wgs84_utils::wgs84_coordinate &loc, tf2::Transform ecef_in_ned) {

    // frame2ecefTransform needs to define the position of the ecefFrame relative to the desired frame
    // Put geodesic in proper units
    double lonRad = loc.lon;
    double latRad = loc.lat;
    double alt = loc.elevation;

    double sinLat = sin(latRad);
    double sinLon = sin(lonRad);
    double cosLat = cos(latRad);
    double cosLon = cos(lonRad);

    double Ne = Rea / sqrt(1.0 - e_sqr * sinLat * sinLat);// The prime vertical radius of curvature

    double x = (Ne + alt)*cosLat*cosLon;
    double y = (Ne + alt)*cosLat*sinLon;
    double z = (Ne*(1-e_sqr) + alt) * sinLat;

    tf2::Vector3 ecef_point(x,y,z);

    tf2::Vector3 point_in_ned = ecef_in_ned * ecef_point; 
    return point_in_ned;
}

/**
 * Generates a transform describing the location/orientation of an NED frame in the ECEF frame based on the provided lat lon point.
 * 
 * @param loc The location of the NED frame in WGS-84 lat/lon with angles in radians
 * 
 * @return The transform describing the location/orientation of an NED frame in the ECEF frame
 */
tf2::Transform wgs84_utils::ecef_to_ned_from_loc(wgs84_coordinate loc) {

    tf2::Vector3 ecef_point = wgs84_utils::geodesic_to_ecef(loc, tf2::Transform::getIdentity());

    // Rotation matrix of north east down frame with respect to ecef
    // Found at https://en.wikipedia.org/wiki/North_east_down
    double sinLat = sin(loc.lat);
    double sinLon = sin(loc.lon);
    double cosLat = cos(loc.lat);
    double cosLon = cos(loc.lon);

 	  tf2::Matrix3x3 rotMat(
      -sinLat * cosLon, -sinLon,  -cosLat * cosLon,
      -sinLat * sinLon,  cosLon,  -cosLat * sinLon,
                cosLat,       0,           -sinLat 
    );
    
    return tf2::Transform(rotMat, ecef_point);
  }

wgs84_utils::wgs84_coordinate wgs84_utils::ecef_to_geodesic(const tf2::Vector3& point) {
    double x = point.getX();
    double y = point.getY();
    double z = point.getZ();

    // Calculate lat,lon,alt
    double p = sqrt((x*x) + (y*y));
    // Handle special case of poles
    if (p < 1.0e-10) {
      double poleLat = z < 0 ? -90:90;
      double poleLon = 0;
      double poleAlt = z < 0 ? -z - Reb : z - Reb;
      wgs84_utils::wgs84_coordinate res;
      res.lat = poleLat;
      res.lon = poleLon;
      res.elevation = poleAlt;
      return res;
    }
    double theta = atan((z*Rea) / (p*Reb));

    double lon = 2.0*atan(y / (x + p));
    double lat = atan((z + (e_p * e_p) * Reb * pow(sin(theta), 3)) / (p - e_sqr * Rea * pow(cos(theta), 3)));

    double cosLat = cos(lat);
    double sinLat = sin(lat);

    double N = Rea_sqr / sqrt(Rea_sqr * cosLat * cosLat + Reb_sqr * sinLat * sinLat);
    double alt = (p / cosLat) - N;
    wgs84_utils::wgs84_coordinate res;
    res.lat = RAD2DEG* lat;
    res.lon = RAD2DEG * lon;
    res.elevation = alt;
    return res;

}