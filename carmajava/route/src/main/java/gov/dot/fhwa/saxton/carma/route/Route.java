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

package gov.dot.fhwa.saxton.carma.route;

import org.ros.message.MessageFactory;
import org.ros.message.Time;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Class which represents the abstraction of a travel route which a vehicle will follow.
 * A routes geometry is defined by RouteWaypoints which has lat/lon coordinates.
 */
public class Route {
  protected Time expectedTimeOfArrival;
  protected String routeID;
  protected String routeName;
  protected double routeLength;
  protected double maxJoinDistance = 20.0;
  protected List<RouteSegment> segments;
  protected List<RouteWaypoint> waypoints;

  /**
   * Default constructor does nothing.
   * Needed to make the route a bean which can be easily parsed from yaml file
   */
  public Route() {}

  /**
   * Constructor which initializes a route from a provided list of waypoints
   *
   * @param waypoints The list of waypoints which will be used to build the route
   * @param routeID   The id to assign to the route. Should be unique
   * @param routeName The display name of the route
   */
  public Route(List<RouteWaypoint> waypoints, String routeID, String routeName) {
    this.routeID = routeID;
    this.routeName = routeName;
    this.setWaypoints(waypoints);
  }

  /**
   * Constructs a ros message from this route
   *
   * @param factory The message factory which will be used to get a ros message object
   * @return A route message with all fields set except the std_msgs.Header
   */
  public cav_msgs.Route toMessage(MessageFactory factory) {
    cav_msgs.Route routeMsg = factory.newFromType(cav_msgs.Route._TYPE);
    routeMsg.setRouteID(routeID);
    routeMsg.setRouteName(routeName);

    List<cav_msgs.RouteSegment> routeSegmentMsgs = new LinkedList<>();
    for (int i = 0; i < segments.size(); i++) {
      routeSegmentMsgs.add(segments.get(i).toMessage(factory, i + 1));
    }

    routeMsg.setSegments(routeSegmentMsgs);
    return routeMsg;
  }

  /**
   * Converts a ros message into an initialized Route object
   * @param routeMsg The ros message
   * @return The route object
   */
  public static Route fromMessage(cav_msgs.Route routeMsg){
    List<RouteWaypoint> waypoints = new LinkedList<>();

    for (cav_msgs.RouteSegment segmentMsg: routeMsg.getSegments()){
      RouteSegment segment = RouteSegment.fromMessage(segmentMsg);
      waypoints.add(segment.getUptrackWaypoint());
      waypoints.add(segment.getDowntrackWaypoint());
    }
    return new Route(waypoints, routeMsg.getRouteID(), routeMsg.getRouteName());
  }

  /**
   * Calculates the length of a route
   * @return the length of the route in meters
   */
  protected void calculateLength(){
    double totalLength = 0;

    for(RouteSegment seg: segments) {
      totalLength += seg.length();
    }
    this.routeLength = totalLength;
  }

  /**
   * Calculates the distance downtrack to the end of the segment with the specified index.
   * The calculation is performed from the start of the segment with the specified startIndex
   * @param startIndex the index of the first segment to be included in the length calculation
   * @param finalIndex the index of the final segment to be included in the length calculation
   * @return the length of the route in meters
   */
  public double lengthOfSegments(int startIndex, int finalIndex){
    double totalLength = 0;

    for(int i = startIndex; i <= finalIndex; i++) {
      totalLength += segments.get(i).length();
    }

    return totalLength;
  }

  /**
   * Inserts the provided waypoint into the route at the specified index.
   * The waypoint currently at that index will be right shifted (placed at index + 1)
   * Inserting a waypoint will result in an additional route segment being created.
   * To insert at the end of the list use an index = waypoints.size()
   * To insert at the front of the list use 0.
   * Out of bound indexes will be truncated to 0 or waypoints.size() usable values
   *
   * @param waypoint The RouteWaypoint to be inserted. Must be able to connect to previous and next waypoints
   * @param index    The index at which to insert the RouteWaypoint. Inserting at a non-existent index will result in an exception.
   * @return Returns true if the waypoint was inserted successfully. False otherwise.
   */
  public boolean insertWaypoint(RouteWaypoint waypoint, int index) throws IndexOutOfBoundsException{
    //TODO perform validation check on waypoint usability
    // Remove the segment at that location and replace it with two segments connected to the new waypoint
    // If waypoint not inserted at the front or end of the list the existing segments must be modified
    if (index != 0 && index < waypoints.size()) {
      segments.remove(index-1);
      segments.add(index-1, new RouteSegment(waypoints.get(index - 1), waypoint));
      segments.add(index, new RouteSegment(waypoint, waypoints.get(index)));
    } else if (index == waypoints.size() && waypoint.getWaypointLaneIndex() == waypoints.get(index-1).getWaypointLaneIndex()) {
      segments.add(new RouteSegment(waypoints.get(index-1), waypoint));
    } else if (index == 0 && waypoint.getWaypointLaneIndex() == waypoints.get(index-1).getWaypointLaneIndex()){
      segments.add(index, new RouteSegment(waypoint, waypoints.get(index)));
    } else {
      throw new IndexOutOfBoundsException("Failed to add " + waypoint + " at index: " + index);
    }

    // Insert the waypoint into the list of waypoints
    waypoints.add(index,waypoint);

    calculateLength();
    return true;
    //return false;
  }

  /**
   * Gets the first (starting) segment of this route
   *
   * @return The route segment which is the first segment of the route
   */
  public RouteSegment getFirstSegment() {
    return segments.get(0);
  }

  /**
   * Gets the last (ending) segment of this route
   *
   * @return The route segment which is the first segment of the route
   */
  public RouteSegment getLastSegment() {
    return segments.get(segments.size()-1);
  }

  /**
   * Gets an immutable list of segments.
   * To modify the segments list the insert segment method should be used.
   *
   * @return The immutable ist of segments generated with Collections.unmodifiableList()
   */
  public List<RouteSegment> getSegments() {
    return Collections.unmodifiableList(segments);
  }

  /**
   * Gets the route name
   *
   * @return the name of this route
   */
  public String getRouteName() {
    return routeName;
  }

  /**
   * Sets the route name
   *
   * @param routeName The name which will be used for this route
   */
  public void setRouteName(String routeName) {
    this.routeName = routeName;
  }

  /**
   * Gets an immutable list of waypoints.
   * To modify waypoints the insert waypoint method should be used.
   *
   * @return The immutable ist of waypoints generated with Collections.unmodifiableList()
   */
  public List<RouteWaypoint> getWaypoints() {
    return Collections.unmodifiableList(waypoints);
  }

  /**
   * Sets the list of waypoints
   *
   * @param waypointList The list of waypoints which will be assigned
   */
  public void setWaypoints(List<RouteWaypoint> waypointList) {
    waypoints = waypointList;
    boolean firstWaypoint = true;
    RouteWaypoint prevWaypoint = null;
    // Build segments from waypoints
    segments = new LinkedList<>(); // Clear currnet waypoints

    for(RouteWaypoint waypoint: waypointList){

      if (!firstWaypoint && prevWaypoint.getWaypointLaneIndex() == waypoint.getWaypointLaneIndex()){
        segments.add(new RouteSegment(prevWaypoint, waypoint));
      }
      prevWaypoint = waypoint;
      firstWaypoint = false;
    }
    calculateLength();
  }

  /**
   * Gets the expected time of arrival for a vehicle on this route
   *
   * @return The expected time of arrival
   */
  public Time getExpectedTimeOfArrival() {
    return expectedTimeOfArrival;
  }

  /**
   * Gets the id of this route
   *
   * @return the id
   */
  public String getRouteID() {
    return routeID;
  }

  /**
   * Sets the route id
   *
   * @param routeID the route id which will be assigned
   */
  public void setRouteID(String routeID) {
    this.routeID = routeID;
  }

  /**
   * Gets the length of this route in meters
   *
   * @return length of the route
   */
  public double getRouteLength() {
    return routeLength;
  }

  /**
   * Get the maximum distance which a vehicle can be from a route and still join it
   * @return distance in meters
   */
  public double getMaxJoinDistance() {
    return maxJoinDistance;
  }

  /**
   * Set the max distance which a vehicle can be from a route and still join it
   * @param maxJoinDistance The max distance in meters
   */
  public void setMaxJoinDistance(double maxJoinDistance) {
    this.maxJoinDistance = maxJoinDistance;
  }
}
