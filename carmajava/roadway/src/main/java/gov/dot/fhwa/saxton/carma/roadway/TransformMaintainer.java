/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.roadway;

import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import org.apache.commons.logging.Log;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import std_msgs.Header;
import tf2_msgs.TFMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * The TransformMaintainer if responsible for updating coordinate transforms
 */
public class TransformMaintainer {
  // Messaging and logging
  protected SaxtonLogger log;
  protected IRoadwayManager roadwayMgr;
  protected final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

  // Host vehicle state variables
  protected boolean headingReceived = false;
  protected boolean navSatFixReceived = false;
  protected Location hostVehicleLocation = null;
  protected double hostVehicleHeading;
  // The heading of the vehicle in degrees east of north in an NED frame.
  // Frame ids
  protected final String earthFrame;
  protected final String mapFrame;
  protected final String odomFrame;
  protected final String baseLinkFrame;
  protected final String globalPositionSensorFrame;
  protected final String localPositionSensorFrame;

  // Transforms
  protected Transform mapToOdom = Transform.identity();
  protected Transform earthToMap = null;
  protected Transform baseToLocalPositionSensor = null;
  protected Transform baseToGlobalPositionSensor = null;
  protected Transform odomToBaseLink = Transform.identity();// The odom frame will start in the same orientation as the base_link frame on startup
  // Transform Update parameters
  protected Time prevMapTime = null;
  protected Duration MAP_UPDATE_PERIOD = new Duration(5); // TODO Time in seconds between updating the map frame location
  protected int tfSequenceCount = 0;

  /**
   * Constructor
   *
   * @param roadwayMgr EnvironmentWorker used to publish data and get stored transforms
   * @param log    Logging object
   * @param earthFrame The frame id used to identify the ECEF frame
   * @param mapFrame The frame id used to identify the global map frame
   * @param odomFrame The frame id used to identify the local planning frame
   * @param baseLinkFrame The frame id used to identify the host vehicle frame
   * @param globalPositionSensorFrame The frame id used to identify the frame of a global position sensor
   * @param localPositionSensorFrame The frame id used to identify a local odometry position sensor frame
   */
  public TransformMaintainer(IRoadwayManager roadwayMgr, Log log, String earthFrame, String mapFrame,
    String odomFrame, String baseLinkFrame, String globalPositionSensorFrame, String localPositionSensorFrame) {
    this.log = new SaxtonLogger(this.getClass().getSimpleName(), log);
    this.roadwayMgr = roadwayMgr;
    this.earthFrame = earthFrame;
    this.mapFrame = mapFrame;
    this.odomFrame = odomFrame;
    this.baseLinkFrame = baseLinkFrame;
    this.globalPositionSensorFrame = globalPositionSensorFrame;
    this.localPositionSensorFrame = localPositionSensorFrame;
  }

  /**
   * Handler for new vehicle heading messages
   * Headings should be specified as degrees east of north
   * TODO if base_link frame is not an +X forward frame this will need a transform as well.
   *
   * @param heading The heading message
   */
  public void handleHeadingMsg(cav_msgs.HeadingStamped heading) {
    hostVehicleHeading = heading.getHeading();
    headingReceived = true;
  }

  /**
   * NavSatFix Handler
   * Updates the host vehicle's location and updates the earth->map and map->odom transforms
   *
   * @param navSatFix The nav sat fix message
   */
  public void handleNavSatFixMsg(sensor_msgs.NavSatFix navSatFix) {
    // Assign the new host vehicle location
    hostVehicleLocation =
      new Location(navSatFix.getLatitude(), navSatFix.getLongitude(), navSatFix.getAltitude());
    navSatFixReceived = true;
    String frameId = navSatFix.getHeader().getFrameId();
    if (!frameId.equals(globalPositionSensorFrame)) {
      String msg = "NavSatFix message with unsupported frame received. Frame: " + frameId;
      log.fatal("TRANSFORM", msg);
      throw new IllegalArgumentException(msg);
    }
    updateMapAndOdomTFs();
  }

  /**
   * Helper function for use in handleNavSatFix
   * Updates the earth->map and map->odom transforms based on the current vehicle odometry, lat/lon, and heading.
   * For full functionality at least one nav sat fix and heading message needs to have been received.
   * Additionally, a transform from base_link to position_sensor needs to be available
   */
  protected void updateMapAndOdomTFs() {
    if (!navSatFixReceived || !headingReceived) {
      return; // If we don't have a heading and a nav sat fix the map->odom transform cannot be calculated
    }
    // Check if base_link->position_sensor tf is available. If not look it up
    if (baseToGlobalPositionSensor == null) {
      // This transform should be static. No need to look up more than once
      baseToGlobalPositionSensor =
        roadwayMgr.getTransform(baseLinkFrame, globalPositionSensorFrame, Time.fromMillis(0));
      if (baseToGlobalPositionSensor == null) {
        return; // If the request for this transform failed wait for another position update to request it
      }
    }

    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

    List<geometry_msgs.TransformStamped> tfStampedMsgs = new LinkedList<>();

    // Update map location on start
    if (prevMapTime == null) { // TODO Determine earth->map update method || 0 < roadwayMgr.getTime().subtract(prevMapTime).compareTo(MAP_UPDATE_PERIOD)) {
      // Map will be an NED frame on the current vehicle location
      earthToMap = gcc.ecefToNEDFromLocaton(hostVehicleLocation);
      prevMapTime = roadwayMgr.getTime();
    }
    // Keep publishing transform to maintain timestamp
    tfStampedMsgs.add(buildTFStamped(earthToMap, earthFrame, mapFrame, roadwayMgr.getTime()));

    // Calculate map->global_position_sensor transform
    Point3D globalSensorInMap = gcc.geodesic2Cartesian(hostVehicleLocation, earthToMap.invert());

    // T_x_y = transform describing location of y with respect to x
    // m = map frame
    // b = baselink frame (from odometry)
    // B = baselink frame (from nav sat fix)
    // o = odom frame
    // p = global position sensor frame
    // We want to find T_m_o. This is the new transform from map to odom.
    // T_m_o = T_m_B * inv(T_o_b)  since b and B are both odom.
    Vector3 nTranslation = new Vector3(globalSensorInMap.getX(), globalSensorInMap.getY(), globalSensorInMap.getZ());
    // The vehicle heading is relative to NED so over short distances heading in NED = heading in map
    Vector3 zAxis = new Vector3(0, 0, 1);
    Quaternion globalSensorRotInMap = Quaternion.fromAxisAngle(zAxis, Math.toRadians(hostVehicleHeading));
    globalSensorRotInMap = globalSensorRotInMap.normalize();

    Transform T_m_p = new Transform(nTranslation, globalSensorRotInMap);
    Transform T_B_p = baseToGlobalPositionSensor;
    Transform T_m_B = T_m_p.multiply(T_B_p.invert());
    Transform T_o_b = odomToBaseLink;

    // Modify map to odom with the difference from the expected and real sensor positions
    mapToOdom = T_m_B.multiply(T_o_b.invert());
    // Publish newly calculated transforms
    tfStampedMsgs.add(buildTFStamped(mapToOdom, mapFrame, odomFrame, roadwayMgr.getTime()));
    publishTF(tfStampedMsgs);
  }

  /**
   * Odometry message handler
   *
   * @param odometry Odometry message
   */
  public void handleOdometryMsg(nav_msgs.Odometry odometry) {
    // Check if base_link->position_sensor tf is available. If not look it up
    if (baseToLocalPositionSensor == null) {
      // This transform should be static. No need to look up more than once
      baseToLocalPositionSensor =
        roadwayMgr.getTransform(baseLinkFrame, localPositionSensorFrame, Time.fromMillis(0));
      if (baseToLocalPositionSensor == null) {
        return; // If the request for this transform failed wait for another odometry update to request it
      }
    }

    String parentFrameId = odometry.getHeader().getFrameId();
    String childFrameId = odometry.getChildFrameId();
    // If the odometry is already in the base_link frame
    if (parentFrameId.equals(odomFrame) && childFrameId.equals(baseLinkFrame)) {
      odomToBaseLink = Transform.fromPoseMessage(odometry.getPose().getPose());
      publishTF(Arrays.asList(buildTFStamped(odomToBaseLink, odomFrame, baseLinkFrame, roadwayMgr.getTime())));

    } else if (parentFrameId.equals(odomFrame) && childFrameId.equals(localPositionSensorFrame)) {
      // Extract the location of the position sensor relative to the odom frame
      // Covariance is ignored as filtering was already done by sensor fusion
      // Calculate odom->base_link
      // T_x_y = transform describing location of y with respect to x
      // p = position sensor frame (from odometry)
      // o = odom frame
      // b = baselink frame (as has been calculated by odometry up to this point)
      // T_o_b = T_o_p * inv(T_b_p)
      Transform T_o_p = Transform.fromPoseMessage(odometry.getPose().getPose());
      Transform T_b_p = baseToLocalPositionSensor;
      Transform T_o_b = T_o_p.multiply(T_b_p.invert());
      odomToBaseLink = T_o_b;
      // Publish updated transform
      publishTF(Arrays.asList(buildTFStamped(odomToBaseLink, odomFrame, baseLinkFrame, roadwayMgr.getTime())));

    } else {
      String msg =
        "Odometry message with unsupported frames received. ParentFrame: " + parentFrameId
          + " ChildFrame: " + childFrameId;
      log.fatal("TRANSFORM", msg);
      throw new IllegalArgumentException(msg);
    }
  }

  /**
   * Velocity message handler TODO decide if this is needed
   *
   * @param velocity host vehicle velocity
   */
  public void handleVelocityMsg(geometry_msgs.TwistStamped velocity) {
    //TODO update host vehicle specification
  }

  /**
   * Helper function builds a tf2 message for the given transform between parent and child frames
   *
   * @param tf          The transform to publish. Describes the position of the child frame in the parent frame
   * @param parentFrame The name of the parent frame
   * @param childFrame  The name of the child frame
   * @param stamp The timestamp of this transform
   */
  protected geometry_msgs.TransformStamped buildTFStamped(Transform tf, String parentFrame,
    String childFrame, Time stamp) {
    geometry_msgs.TransformStamped tfStampedMsg =
      messageFactory.newFromType(geometry_msgs.TransformStamped._TYPE);
    Header hdr = tfStampedMsg.getHeader();
    hdr.setFrameId(parentFrame);
    hdr.setStamp(roadwayMgr.getTime());
    hdr.setSeq(tfSequenceCount);
    hdr.setStamp(stamp);
    tfStampedMsg.setChildFrameId(childFrame);
    tfStampedMsg.setTransform(tf.toTransformMessage(tfStampedMsg.getTransform()));
    return tfStampedMsg;
  }

  /**
   * Publishes a list of transforms in a single tf2 TFMessage
   *
   * @param tfList List of transforms
   */
  protected void publishTF(List<TransformStamped> tfList) {
    TFMessage tfMsg = messageFactory.newFromType(TFMessage._TYPE);
    tfMsg.setTransforms(tfList);
    tfSequenceCount++;
    roadwayMgr.publishTF(tfMsg);
  }
}
