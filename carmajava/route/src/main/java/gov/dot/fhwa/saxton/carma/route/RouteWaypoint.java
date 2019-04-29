/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.Maneuver;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Waypoints are representations of sequential regions along a route.
 * Each waypoint has a latitude and longitude and contains the road specifications for that location.
 * Such specifications include speedlimit, number of lanes, lane closures, needed maneuvers, nearest mile markers, and required lanes.
 * Lane indices will run right to left. Such that the right most lane will be lane 0.
 * Crosstrack sign convention is that positive indicates an offset to the right of segment centerline.
 */
public class RouteWaypoint {
  protected int waypointId; // The waypoint id is only set when a waypoint is published. It corresponds to the waypoint index. It is not used internally
  protected List<String> disabledGuidanceAlgorithms = new LinkedList<>();
  protected List<Integer> laneClosures = new ArrayList<>();
  protected int laneCount = 1;
  protected Location location;
  protected int lowerSpeedLimit = 0; // Units: mph
  protected int upperSpeedLimit = 5; // Units: mph
  protected double nearestMileMarker = -1;
  protected List<cav_msgs.Maneuver> neededManeuvers = new LinkedList<>();
  protected double minCrossTrack = -10.0; // Units: m
  protected double maxCrossTrack = 10.0; // Units: m
  protected int requiredLaneIndex = -1;
  protected int laneIndex = 0;
  protected double laneWidth = 3.7; // Units: m
  protected Point3D ecefPoint;
  protected RoadType roadType = RoadType.FREEWAY;
  protected LaneEdgeType interiorLaneMarkings = LaneEdgeType.SOLID_WHITE;
  protected LaneEdgeType leftMostLaneMarking = LaneEdgeType.SOLID_YELLOW;
  protected LaneEdgeType rightMostLaneMarking = LaneEdgeType.SOLID_WHITE;
  protected GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

  /**
   * Default constructor
   */
  public RouteWaypoint(){}

  /**
   * Constructor initializes this waypoint with the specified location and a speed limit range of [0,speedLimit]
   *
   * @param loc The gps location of this waypoint.
   */
  public RouteWaypoint(Location loc) {
    setLocation(loc);
    this.lowerSpeedLimit = 0;
    this.upperSpeedLimit = 5;
  }

  /**
   * Deep copy constructor
   */
  public RouteWaypoint(RouteWaypoint wp) {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    // String and Integer are immutable so add basic copy will work
    disabledGuidanceAlgorithms.addAll(wp.disabledGuidanceAlgorithms);
    laneClosures.addAll(wp.laneClosures);
    // Copy remaining fields
    laneCount = wp.laneCount;
    this.setLocation(wp.getLocation());
    lowerSpeedLimit = wp.lowerSpeedLimit;
    upperSpeedLimit = wp.upperSpeedLimit;
    nearestMileMarker = wp.nearestMileMarker;
    minCrossTrack = wp.minCrossTrack;
    maxCrossTrack = wp.maxCrossTrack;
    requiredLaneIndex = wp.requiredLaneIndex;
    laneIndex = wp.laneIndex;
    laneWidth = wp.laneWidth;
    roadType = wp.roadType;
    interiorLaneMarkings = wp.interiorLaneMarkings;
    leftMostLaneMarking = wp.leftMostLaneMarking;
    rightMostLaneMarking = wp.rightMostLaneMarking;
  }

  /**
   * Gets the waypoint id
   * @return waypoint id
   */
  public int getWaypointId() {
    return this.waypointId;
  }

  /**
   * Sets the waypoint id
   * @param waypointId the waypoint id
   */
  public void setWaypointId(int waypointId) {
    this.waypointId = waypointId;
  }

  /**
   * Generates a set of bit flags denoting which fields have been set for this waypoint
   * Fields are evaluated by comparing them to null or the default value of -1
   *
   * @return a 16bit mask denoting the set fields according to the cav_msgs.RouteWaypoint specification
   */
  public short getSetFields() {
    int bitMask = 0x0000;
    if (!disabledGuidanceAlgorithms.isEmpty())
      bitMask = bitMask | cav_msgs.RouteWaypoint.DISABLED_GUIDANCE_ALGORITHMS;
    if (!laneClosures.isEmpty())
      bitMask = bitMask | cav_msgs.RouteWaypoint.LANE_CLOSURES;
    if (nearestMileMarker != -1)
      bitMask = bitMask | cav_msgs.RouteWaypoint.NEAREST_MILE_MARKER;
    if (!neededManeuvers.isEmpty())
      bitMask = bitMask | cav_msgs.RouteWaypoint.NEEDED_MANEUVERS;
    if (requiredLaneIndex != -1)
      bitMask = bitMask | cav_msgs.RouteWaypoint.REQUIRED_LANE_INDEX;
    if (roadType != null)
      bitMask = bitMask | cav_msgs.RouteWaypoint.ROAD_TYPE;

    return (short) bitMask;
  }

  /**
   * Constructs a fully initialized ros message from this route waypoint
   *
   * @param factory The message factory which will be used to get a ros message object
   * @return The ros message
   */
  public cav_msgs.RouteWaypoint toMessage(MessageFactory factory, int waypointIndex) {
    cav_msgs.RouteWaypoint routeWPMsg = factory.newFromType(cav_msgs.RouteWaypoint._TYPE);
    routeWPMsg.setWaypointId(waypointIndex);
    routeWPMsg.setSpeedLimit((byte) upperSpeedLimit); //TODO clarify this
    routeWPMsg.setSetFields(getSetFields());

    cav_msgs.RoadType roadTypeMsg = factory.newFromType(cav_msgs.RoadType._TYPE);
    roadTypeMsg.setType((byte) roadType.ordinal());
    routeWPMsg.setRoadType(roadTypeMsg);
    routeWPMsg.setRequiredLaneIndex((byte) requiredLaneIndex);
    routeWPMsg.setLaneIndex((byte) laneIndex);
    routeWPMsg.setNearestMileMarker((byte) nearestMileMarker);
    routeWPMsg.setLongitude(location.getLongitude());
    routeWPMsg.setLatitude(location.getLatitude());
    routeWPMsg.setAltitude(location.getAltitude());
    routeWPMsg.setMinCrossTrack(minCrossTrack);
    routeWPMsg.setMaxCrossTrack(maxCrossTrack);
    routeWPMsg.setLaneCount((byte) laneCount);
    routeWPMsg.setDisabledGuidanceAlgorithms(disabledGuidanceAlgorithms);
    routeWPMsg.setNeededManeuvers(neededManeuvers);
    routeWPMsg.setInteriorLaneMarkings(interiorLaneMarkings.toMessage());
    routeWPMsg.setLeftMostLaneMarking(leftMostLaneMarking.toMessage());
    routeWPMsg.setRightMostLaneMarking(rightMostLaneMarking.toMessage());
    routeWPMsg.setLaneWidth((float) laneWidth);

    byte[] laneClosuresAsBytes = new byte[laneClosures.size()];
    for (int i = 0; i < laneClosures.size(); i++) {
      laneClosuresAsBytes[i] = laneClosures.get(i).byteValue();
    }

    // It seems that the ros messages byte[] is LittleEndian. Using BigEndian results in a IllegalArgumentException
    if (laneClosuresAsBytes.length > 0) {
      routeWPMsg
        .setLaneClosures(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, laneClosuresAsBytes));
    }

    return routeWPMsg;
  }

  /**
   * Constructs a fully initialized RouteWaypoint object from a ros message
   *
   * @param waypointMsg The ros message
   * @return The RouteWaypoint object
   */
  public static RouteWaypoint fromMessage(cav_msgs.RouteWaypoint waypointMsg) {
    RouteWaypoint wp = new RouteWaypoint(
      new Location(waypointMsg.getLatitude(), waypointMsg.getLongitude(),
        waypointMsg.getAltitude()));
    wp.setWaypointId(waypointMsg.getWaypointId());
    wp.setLaneCount(waypointMsg.getLaneCount());
    wp.setLowerSpeedLimit(waypointMsg.getLowerSpeedLimit());
    wp.setUpperSpeedLimit(waypointMsg.getSpeedLimit());
    wp.setLeftMostLaneMarking(LaneEdgeType.fromMessge(waypointMsg.getLeftMostLaneMarking()));
    wp.setInteriorLaneMarkings(LaneEdgeType.fromMessge(waypointMsg.getInteriorLaneMarkings()));
    wp.setRightMostLaneMarking(LaneEdgeType.fromMessge(waypointMsg.getRightMostLaneMarking()));
    wp.setMinCrossTrack(waypointMsg.getMinCrossTrack());
    wp.setMaxCrossTrack(waypointMsg.getMaxCrossTrack());
    wp.laneIndex = waypointMsg.getLaneIndex();
    wp.laneWidth = waypointMsg.getLaneWidth();

    // Set member variables according to set fields bit mask
    int bitMask = waypointMsg.getSetFields();
    if ((bitMask & cav_msgs.RouteWaypoint.DISABLED_GUIDANCE_ALGORITHMS) != 0)
      wp.setDisabledGuidanceAlgorithms(waypointMsg.getDisabledGuidanceAlgorithms());
    if ((bitMask & cav_msgs.RouteWaypoint.LANE_CLOSURES) != 0) {
      ChannelBuffer buffer = waypointMsg.getLaneClosures();
      List<Integer> laneClosures = new LinkedList<>();
      for (byte b : buffer.array()) {
        laneClosures.add(Integer.valueOf(b));
      }
      wp.setLaneClosures(laneClosures);
    }
    if ((bitMask & cav_msgs.RouteWaypoint.NEAREST_MILE_MARKER) != 0)
      wp.setNearestMileMarker(waypointMsg.getNearestMileMarker());
    if ((bitMask & cav_msgs.RouteWaypoint.NEEDED_MANEUVERS) != 0)
      wp.setNeededManeuvers(waypointMsg.getNeededManeuvers());
    if ((bitMask & cav_msgs.RouteWaypoint.REQUIRED_LANE_INDEX) != 0)
      wp.setRequiredLaneIndex(waypointMsg.getRequiredLaneIndex());
    if ((bitMask & cav_msgs.RouteWaypoint.ROAD_TYPE) != 0)
      wp.setRoadType(RoadType.fromMessage(waypointMsg.getRoadType()));

    return wp;
  }

  /**
   * Sets the road type of this waypoint
   *
   * @param roadType the road type to set
   */
  public void setRoadType(RoadType roadType) {
    this.roadType = roadType;
  }

  /**
   * Gets the road type of this waypoint
   *
   * @return the road type
   */
  public RoadType getRoadType() {
    return roadType;
  }

  /**
   * Sets the gps location of this waypoint
   *
   * @param location the gps location
   */
  public void setLocation(Location location) {
    this.location = location;
    this.ecefPoint = gcc.geodesic2Cartesian(location, Transform.identity());
  }

  /**
   * Gets the gps location of this waypoint
   *
   * @return the gps location
   */
  public Location getLocation() {
    return location;
  }

  /**
   * Gets the minimum cross track value a vehicle can have along the segment defined by this waypoint
   * @return The min cross track distance in meters
   */
  public double getMinCrossTrack() {
    return minCrossTrack;
  }

  /**
   * Sets the minimum cross track value a vehicle can have along the segment defined by this waypoint
   * @param minCrossTrack The min cross track distance in meters
   */
  public void setMinCrossTrack(double minCrossTrack) {
    this.minCrossTrack = minCrossTrack;
  }

  /**
   * Gets the maximum cross track value a vehicle can have along the segment defined by this waypoint
   * @return The max cross track distance in meters
   */
  public double getMaxCrossTrack() {
    return maxCrossTrack;
  }

  /**
   * Sets the maximum cross track value a vehicle can have along the segment defined by this waypoint
   * @param maxCrossTrack The max cross track distance in meters
   */
  public void setMaxCrossTrack(double maxCrossTrack) {
    this.maxCrossTrack = maxCrossTrack;
  }

  /**
   * Sets the list of disabled guidance algorithms
   *
   * @param disabledGuidanceAlgorithms list of guidance algorithm names
   */
  public void setDisabledGuidanceAlgorithms(List<String> disabledGuidanceAlgorithms) {
    this.disabledGuidanceAlgorithms = disabledGuidanceAlgorithms;
  }

  /**
   * Gets a list of disabled guidance algorithms at this waypoint
   *
   * @return List of algorithm names
   */
  public List<String> getDisabledGuidanceAlgorithms() {
    return disabledGuidanceAlgorithms;
  }

  /**
   * Sets the list of closed lane indices
   *
   * @param laneClosures list of lane indices
   */
  public void setLaneClosures(List<Integer> laneClosures) {
    this.laneClosures = laneClosures;
  }

  /**
   * Gets a list of lane closures where lanes are identified by index
   *
   * @return the list of lane indexes
   */
  public List<Integer> getLaneClosures() {
    return laneClosures;
  }

  /**
   * Sets the total number of lanes at this waypoint
   *
   * @param laneCount the lane count
   */
  public void setLaneCount(int laneCount) {
    this.laneCount = laneCount;
  }

  /**
   * Gets the total number of lanes at this waypoint
   *
   * @return The lane count
   */
  public int getLaneCount() {
    return laneCount;
  }

  /**
   * Sets the lower speed limit at this waypoint
   *
   * @param lowerSpeedLimit the lower speed limit
   */
  public void setLowerSpeedLimit(int lowerSpeedLimit) {
    this.lowerSpeedLimit = lowerSpeedLimit;
  }

  /**
   * Gets the lower speed limit at this waypoint
   *
   * @return the lower speed limit
   */
  public int getLowerSpeedLimit() {
    return lowerSpeedLimit;
  }

  /**
   * Sets the upper speed limit at this waypoint
   *
   * @param upperSpeedLimit the upper speed limit
   */
  public void setUpperSpeedLimit(int upperSpeedLimit) {
    this.upperSpeedLimit = upperSpeedLimit;
  }

  /**
   * Gets the upper speed limit at this waypoint
   *
   * @return the upper speed limit
   */
  public int getUpperSpeedLimit() {
    return upperSpeedLimit;
  }

  /**
   * Sets the value of the nearest mile marker generally with only 10^-1 precision
   *
   * @param nearestMileMarker the nearest mile marker value
   */
  public void setNearestMileMarker(double nearestMileMarker) {
    this.nearestMileMarker = nearestMileMarker;
  }

  /**
   * Gets the nearest mile marker at this waypoint
   *
   * @return the mile marker
   */
  public double getNearestMileMarker() {
    return nearestMileMarker;
  }

  /**
   * Sets the required lane index at this waypoint
   *
   * @param requiredLaneIndex lane index
   */
  public void setRequiredLaneIndex(int requiredLaneIndex) {
    this.requiredLaneIndex = requiredLaneIndex;
  }

  /**
   * Gets the index of the lane which the vehicle must be in to continue following a route using this waypoint
   *
   * @return the required lane index
   */
  public int getRequiredLaneIndex() {
    return requiredLaneIndex;
  }

  /**
   * Sets the index of the lane which this waypoint is in
   *
   * @param laneIndex lane index
   */
  public void setLaneIndex(int laneIndex) {
    this.laneIndex = laneIndex;
  }

  /**
   * Gets the index of the lane which this waypoint is in
   *
   * @return the waypoint lane index
   */
  public int getLaneIndex() {
    return laneIndex;
  }

  /**
   * Sets the width of the lane which this waypoint is in
   *
   * @param laneIndex lane width in m
   */
  public void setLaneWidth(double laneWidth) {
    this.laneWidth = laneWidth;
  }

  /**
   * Gets the lane width in m at this waypoint
   *
   * @return the waypoint lane width in m
   */
  public double getLaneWidth() {
    return laneWidth;
  }

  /**
   * Gets the type of lane marking which separates interior lanes on this segment
   * @return the interior lane marking type
   */
  public LaneEdgeType getInteriorLaneMarkings() {
    return interiorLaneMarkings;
  }

  /**
   * Sets the type of lane marking which separates interior lanes on this segment
   */
  public void setInteriorLaneMarkings(LaneEdgeType interiorLaneMarkings) {
    this.interiorLaneMarkings = interiorLaneMarkings;
  }

  /**
   * Gets the type of lane marking on the left side of the left most lane
   * @return the lane edge type
   */
  public LaneEdgeType getLeftMostLaneMarking() {
    return leftMostLaneMarking;
  }

  /**
   * Sets the type of lane marking on the right side of the right most lane
   */
  public void setLeftMostLaneMarking(LaneEdgeType leftMostLaneMarking) {
    this.leftMostLaneMarking = leftMostLaneMarking;
  }

  /**
   * Gets the type of lane marking on the right side of the right most lane
   * @return the lane edge type
   */
  public LaneEdgeType getRightMostLaneMarking() {
    return rightMostLaneMarking;
  }

  /**
   * Sets the type of lane marking on the right side of the right most lane
   */
  public void setRightMostLaneMarking(LaneEdgeType rightMostLaneMarking) {
    this.rightMostLaneMarking = rightMostLaneMarking;
  }

  /**
   * Gets the list of needed maneuvers at this waypoint
   * @return list of maneuvers
   */
  public List<cav_msgs.Maneuver> getNeededManeuvers() {
    return neededManeuvers;
  }

  /**
   * Sets the list of needed maneuvers at this waypoint
   * @param neededManeuvers list of required maneuvers
   */
  public void setNeededManeuvers(List<cav_msgs.Maneuver> neededManeuvers) {
    this.neededManeuvers = neededManeuvers;
  }

  /**
   * Gets the point of this waypoint in an ECEF frame
   * @return point
   */
  public Point3D getECEFPoint() {
    if (ecefPoint == null) {
      ecefPoint = gcc.geodesic2Cartesian(location, Transform.identity());
    }
    return ecefPoint;
  }

  @Override public String toString() {
    return "Waypoint{ " + location.toString() + " LaneIndex: " + laneIndex + " }";
  }
}
