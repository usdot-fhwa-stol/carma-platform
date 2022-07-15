#pragma once

/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <functional>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2/LinearMath/Transform.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <boost/optional.hpp>
#include <memory>

#include <gps_msgs/msg/gps_fix.hpp>
#include <wgs84_utils/wgs84_utils.h>

#include <lanelet2_extension/projection/local_frame_projector.h>

/**
 * \class GNSSToMapConvertor
 * \brief Logic class for the GNSSToMapNode which handles the conversion of gnss data to the map frame
 *
 */
namespace gnss_to_map_convertor
{
class GNSSToMapConvertor
{
public:
  using PosePubCallback = std::function<void(geometry_msgs::msg::PoseStamped)>;

  /**
   * Function which will return the most recent transform between the provided frames
   * First frame is target second is source
   * If the transform does not exist or cannot be computed the optional returns false
   */
  using TransformLookupCallback =
      std::function<boost::optional<geometry_msgs::msg::TransformStamped>(const std::string&, const std::string&)>;

  /**
   * \brief Constructor
   *
   * \param pose_pub A function to publish a converted gps fix as a pose to the rest of the system
   * \param tf_lookup A function which will lookup the most recent transform between the two provided frames
   * \param map_frame_id The frame id of the frame which the output pose should be considered in
   * \param base_link_frame_id The frame id of the frame which the output pose defines the position and orientation of
   * \param heading_frame_id The frame id of the frame which the heading report aligns with an NED frame
   * in the map frame.
   *
   */
  GNSSToMapConvertor(PosePubCallback pose_pub, TransformLookupCallback tf_lookup, std::string map_frame_id,
                     std::string base_link_frame_id, std::string heading_frame_id, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);

  /**
   * \brief GNSS Fix callback which will publish a pose representing that fix in the map frame if the required
   * transforms are available
   *
   * \param fix_msg The message to convert to the map frame
   */
  void gnssFixCb(gps_msgs::msg::GPSFix::UniquePtr fix_msg);

  /**
   * \brief Map georeference callback
   *        The geodetic description of the map frame should be provided to this callback as a proj library string.
   *        The frame orientation must be right handed and aligned with lat/lon however the orientation of this
   * alignment does not matter (enu, ned, etc.)
   *
   * \param geo_ref The proj string which defines the geodetic projection of the map frame which is used to convert
   * between GNSS and Map.
   */
  void geoReferenceCallback(std_msgs::msg::String::UniquePtr geo_ref);

  /**
   * \brief Get the rotation computed from the recieved georeference or boost::none if unset
   */
  boost::optional<tf2::Quaternion> getNedInMapRotation();

  /**
   * \brief Get the projector built from the provided georeference via the callback
   */
  std::shared_ptr<lanelet::projection::LocalFrameProjector> getMapProjector();

  /**
   * \brief Converts a provided GNSS fix message into a pose message for the map frame describibed by the provided
   * projector
   *
   * ASSUMPTION:  This logic assumes that the orientation difference between an NED frame located at the map origin and
   * an NED frame located at the GNSS point are sufficiently small that they can be ignored. Therefore it is assumed the
   * heading report of the GNSS system reguardless of its poition in the map without change in its orientation will give
   * the same result (as far as we are concered). This assumption will break down as the distance between the GNSS
   * recieved and the map origin grows. It is recommended this distance be kept under 10km. If larger distances are
   * required then the map origin should probably be periodically updated with a new georeference.
   *
   * \param baselink_in_sensor A transform describing the location of the desried output frame (baselink) with respect
   * to the GNSS sensor frame.
   * \param sensor_in_ned_heading_rotation A rotation describing the orientation of the heading frame with respect to the position sensor frame
   * \param projector A projector using the proj library which can convert lat/lon points into the map frame projection
   * \param ned_in_map_rotation A rotation describibing the orientation of an NED frame located at the map origin with
   * respect to the map frame. \param fix_msg The GNSS message to be converted into the pose in the map frame
   *
   * \return A pose message describing the location and orientation of the baselink frame in the map frame. TODO handle
   * covariance which is not currently included
   */
  geometry_msgs::msg::PoseWithCovarianceStamped poseFromGnss(const tf2::Transform& baselink_in_sensor,
                                                        const tf2::Quaternion& sensor_in_ned_heading_rotation,
                                                        const lanelet::projection::LocalFrameProjector& projector,
                                                        const tf2::Quaternion& ned_in_map_rotation,
                                                        gps_msgs::msg::GPSFix fix_msg);

private:
  PosePubCallback pose_pub_;           // Function which can forward the output from this component
  TransformLookupCallback tf_lookup_;  // Function which accesses a buffer of transforms for query lookups
  std::string map_frame_id_;           // The frame id of the map which the output pose will be in.
  std::string base_link_frame_id_;     // The frame id of the final reported pose
  std::string heading_frame_id_;       // The frame id of the heading frame

  // Rotation describing the orientation of an NED frame relative to the map frame located at the map origin.
  // This is derived from the received georeference
  boost::optional<tf2::Quaternion> ned_in_map_rotation_;

  // Rotation describing orientation of sensor in heading frame
  boost::optional<tf2::Quaternion> sensor_in_ned_heading_rotation_;
  
  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;  // Must be shared pointer instead of
                                                                             // optional since for some reason optional
                                                                             // does not work with this class

  boost::optional<tf2::Transform> baselink_in_sensor_;  // A transform describing the relation of the baselink frame
                                                        // with the frame provided by the gnss message
  
  // Logger interface
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

};  // namespace gnss_to_map_convertor
