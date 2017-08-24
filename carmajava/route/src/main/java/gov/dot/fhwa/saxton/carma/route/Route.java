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
  protected List<RouteSegment> segments;
  protected List<RouteWaypoint> waypoints;

  /**
   * Default constructor does nothing.
   */
  public Route() {
  }

  /**
   * Constructor which initializes a route object from the provided data file.
   *
   * @param filePath The path the the route file which will be loaded
   * @param routeID  The id to assign to the route. Should be unique
   */
  public Route(String filePath, String routeID) {
  }

  /**
   * Constructor which initializes a route from a provided list of waypoints
   *
   * @param waypoints The list of waypoints which will be used to build the route
   * @param routeID   The id to assign to the route. Should be unique
   * @param routeName The display name of the route
   */
  public Route(List<RouteWaypoint> waypoints, String routeID, String routeName) {

  }

  /**
   * Gets the first (starting) segment of this route
   *
   * @return The route segment which is the first segment of the route
   */
  public RouteSegment getFirstSegment() {
    return null;
  }

  /**
   * Gets the last (ending) segment of this route
   *
   * @return The route segment which is the first segment of the route
   */
  public RouteSegment getLastSegment() {
    return null;
  }

  /**
   * Inserts the provided segment into the route at the specified index.
   * The segment currently at that index will be placed before the inserted segment at index-1.
   *
   * @param segment The RouteSegment to be inserted. Must be able to connect to previous and next segments.
   * @param index   The index at which to insert the RouteSegment. Inserting at a non-existent index will result in an exception.
   * @return Returns true if the segment was inserted successfully. False otherwise.
   */
  public boolean insertSegment(RouteSegment segment, int index) {
    return false;
  }

  /**
   * Inserts the provided waypoint into the route at the specified index.
   * The waypoint currently at that index will be placed before the inserted segment at index-1.
   * Inserting a waypoint will result in an additional route segment being created.
   *
   * @param waypoint The RouteWaypoint to be inserted. Must be able to connect to previous and next waypoints
   * @param index    The index at which to insert the RouteWaypoint. Inserting at a non-existent index will result in an exception.
   * @return Returns true if the waypoint was inserted successfully. False otherwise.
   */
  public boolean insertWaypoint(RouteWaypoint waypoint, int index) {
    return false;
  }

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
   * @param waypoints The list of waypoints which will be assigned
   */
  public void setWaypoints(List<RouteWaypoint> waypoints) {
    this.waypoints = waypoints;
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
   * Sets the expected time of arrival.
   *
   * @param expectedTimeOfArrival The expected time of arrival which will be assigned
   */
  public void setExpectedTimeOfArrival(Time expectedTimeOfArrival) {
    this.expectedTimeOfArrival = expectedTimeOfArrival;
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
   * Constructs a ros message from this route
   * @param factory The message factory which will be used to get a ros message object
   * @return A route message with all fields set except the std_msgs.Header
   */
  public cav_msgs.Route toMessage(MessageFactory factory){
    cav_msgs.Route routeMsg = factory.newFromType(cav_msgs.Route._TYPE);
    routeMsg.setRouteID(routeID);
    routeMsg.setRouteName(routeName);

    List<cav_msgs.RouteSegment> routeSegmentMsgs = new LinkedList<>();
    for (int i = 0; i < segments.size(); i++){
      routeSegmentMsgs.add(segments.get(i).toMessage(factory, i+1));
    }

    return routeMsg;
  }
}
