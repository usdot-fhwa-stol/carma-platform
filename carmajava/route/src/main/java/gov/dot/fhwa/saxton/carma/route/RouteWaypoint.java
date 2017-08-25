/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import java.nio.ByteOrder;
import java.util.LinkedList;
import java.util.List;

/**
 * Waypoints are representations of sequential regions along a route.
 * Each waypoint has a latitude and longitude and contains the road specifications for that location.
 * Such specifications include speedlimit, number of lanes, lane closures, needed maneuvers, nearest mile markers, and required lanes.
 * Lane indices will run right to left. Such that the right most lane will be lane 0.
 */
public class RouteWaypoint {
  protected List<String> disabledGuidanceAlgorithms;
  protected List<Integer> laneClosures;
  protected int laneCount = 0;
  protected Location location;
  protected int lowerSpeedLimit = -1;
  protected int upperSpeedLimit = -1;
  protected double nearestMileMarker = -1;
  //protected List<Maneuvers> neededManeuvers; //TODO uncomment when class is implemented
  protected int requiredLaneIndex = -1;
  protected RoadType roadType = RoadType.FREEWAY;

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
    this.location = loc;
    this.lowerSpeedLimit = 0;
    this.upperSpeedLimit = 5;
  }

  /**
   * Generates a set of bit flags denoting which fields have been set for this waypoint
   * Fields are evaluated by comparing them to null or the default value of -1
   *
   * @return a 16bit mask denoting the set fields according to the cav_msgs.RouteWaypoint specification
   */
  public short getSetFields() {
    int bitMask = 0x0000;
    if (disabledGuidanceAlgorithms != null)
      bitMask = bitMask | 0x8000; //1000 0000 0000 0000
    if (laneClosures != null)
      bitMask = bitMask | 0x4000; //0100 0000 0000 0000
    if (laneCount != -1)
      bitMask = bitMask | 0x2000; //0010 0000 0000 0000
    if (nearestMileMarker != -1)
      bitMask = bitMask | 0x1000; //0001 0000 0000 0000
    //  if (needed_manuevers != -1) //TODO uncomment when manuevers class is available
    //    bitMask = bitMaks | 0x0800; //0000 1000 0000 0000
    if (requiredLaneIndex != -1)
      bitMask = bitMask | 0x0400; //0000 0100 0000 0000
    if (roadType != null)
      bitMask = bitMask | 0x0200; //0000 0010 0000 0000
    if (upperSpeedLimit != -1)
      bitMask = bitMask | 0x0100; //0000 0001 0000 0000

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
    routeWPMsg.setNearestMileMarker((byte) nearestMileMarker);
    routeWPMsg.setLongitude(location.getLongitude());
    routeWPMsg.setLatitude(location.getLatitude());
    routeWPMsg.setAltitude(location.getAltitude());
    routeWPMsg.setLaneCount((byte) laneCount);
    routeWPMsg.setDisabledGuidanceAlgorithms(disabledGuidanceAlgorithms);

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

    // Set member variables according to set fields bit mask
    int bitMask = waypointMsg.getSetFields();
    if ((bitMask & 0x8000) != 0) //1000 0000 0000 0000
      wp.setDisabledGuidanceAlgorithms(waypointMsg.getDisabledGuidanceAlgorithms());
    if ((bitMask & 0x4000) != 0) { //0100 0000 0000 0000
      ChannelBuffer buffer = waypointMsg.getLaneClosures();
      List<Integer> laneClosures = new LinkedList<>();
      for (byte b : buffer.array()) {
        laneClosures.add(Integer.valueOf(b));
      }
      wp.setLaneClosures(laneClosures);
    }
    if ((bitMask & 0x2000) != 0) //0010 0000 0000 0000
      wp.setLaneCount(waypointMsg.getLaneCount());
    if ((bitMask & 0x1000) != 0) //0001 0000 0000 0000
      wp.setNearestMileMarker(waypointMsg.getNearestMileMarker());
    //TODO uncomment when manuevers class is available
    //  if ((bitMaks & 0x0800) != 0) //0000 1000 0000 0000
    //    wp.setNeededManeuvers(waypointMsg.getNeededManeuvers());
    if ((bitMask & 0x0400) != 0) //0000 0100 0000 0000
      wp.setRequiredLaneIndex(waypointMsg.getRequiredLaneIndex());
    if ((bitMask & 0x0200) != 0) //0000 0010 0000 0000
      wp.setRoadType(RoadType.fromMessage(waypointMsg.getRoadType()));
    if ((bitMask & 0x0100) != 0) //0000 0001 0000 0000
      wp.setUpperSpeedLimit(waypointMsg.getSpeedLimit());

    // TODO Add lower speed to message specification file
    wp.setLowerSpeedLimit(0);
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
}
