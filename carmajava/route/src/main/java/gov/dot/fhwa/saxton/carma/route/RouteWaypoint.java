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
  protected int laneCount = 1;
  protected Location location;
  protected int lowerSpeedLimit = 0;
  protected int upperSpeedLimit = 5;
  protected float nearestMileMarker;
  //protected List<Manuevers> neededManuevers;
  protected int requiredLaneIndex = -1;
  protected RoadType roadType = RoadType.FREEWAY;

  /**
   * Default constructor does nothing.
   */
  public RouteWaypoint() {}

  /**
   * Constructor intializes this waypoint with the specified location and a speed limit range of [0,speedLimit]
   *
   * @param loc        The gps location of this waypoint.
   * @param speedLimit The upper speed limit at this waypoint specified in m/s
   */
  public RouteWaypoint(Location loc, int speedLimit) {
    this.location = loc;
    this.lowerSpeedLimit = 0;
    this.upperSpeedLimit = speedLimit;
  }

  /**
   * Constructor intializes a new waypoint using the parameters in a ros cav_msgs.RouteWaypoint object.
   *
   * @param waypointMsg The waypoint message object to build this waypoint from
   */
  public RouteWaypoint(cav_msgs.RouteWaypoint waypointMsg) {

  }

  public Location getLocation() {

  }

  public List<String> getDisabledGuidanceAlgorithms() {
    return disabledGuidanceAlgorithms;
  }

  public List<Integer> getLaneClosures() {
    return laneClosures;
  }

  public int getLaneCount() {
    return laneCount;
  }

  public int getLowerSpeedLimit() {
    return lowerSpeedLimit;
  }

  public int getUpperSpeedLimit() {
    return upperSpeedLimit;
  }

  public float getNearestMileMarker() {
    return nearestMileMarker;
  }

  public int getRequiredLaneIndex() {
    return requiredLaneIndex;
  }

  public RoadType getRoadType() {
    return roadType;
  }

  public void setDisabledGuidanceAlgorithms(List<String> disabledGuidanceAlgorithms) {
    this.disabledGuidanceAlgorithms = disabledGuidanceAlgorithms;
  }

  public void setLaneClosures(List<Integer> laneClosures) {
    this.laneClosures = laneClosures;
  }

  public void setLaneCount(int laneCount) {
    this.laneCount = laneCount;
  }

  public void setLowerSpeedLimit(int lowerSpeedLimit) {
    this.lowerSpeedLimit = lowerSpeedLimit;
  }

  public void setUpperSpeedLimit(int upperSpeedLimit) {
    this.upperSpeedLimit = upperSpeedLimit;
  }

  public void setRequiredLaneIndex(int requiredLaneIndex) {
    this.requiredLaneIndex = requiredLaneIndex;
  }
}
