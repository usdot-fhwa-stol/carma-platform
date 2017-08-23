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
 * Class which represents the abstraction of a travel route which a vehicle will follow.
 * A routes geometry is defined by RouteWaypoints which has lat/lon coordinates.
 */
public class Route {
  //Time expectedTimeOfArrival;
  //String routeID;
  public String routeName;
  //double routeLength;
  //List<RouteSegment> segments;
  public List<RouteWaypoint> waypoints;

  /**
   * Default constructor does nothing.
   */
  public Route() {}

  public String getRouteName() {
    return routeName;
  }

  public void setRouteName(String routeName) {
    this.routeName = routeName;
  }

  public List<RouteWaypoint> getWaypoints() {
    return waypoints;
  }

  public void setWaypoints(List<RouteWaypoint> waypoints) {
    this.waypoints = waypoints;
  }

  /**
   * Constructor which initializes a route object from the provided data file.
   *
   * @param filePath The path the the route file which will be loaded
   * @param routeID The id to assign to the route. Should be unique
   */
  public Route(String filePath, String routeID) {
  }

  /**
   * Constructor which initializes a route from a provided list of waypoints
   *
   * @param waypoints The list of waypoints which will be used to build the route
   * @param routeID The id to assign to the route. Should be unique
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

  /**
   * Gets the length of this route in meters.
   *
   * @return The length of this route in meters
   */
  public double length() {
    return 0;
  }

  /**
   * Loads a route into memory using the provided route loading strategy.
   *
   * @param strategy The strategy which will be used to load a route
   * @return True if route loaded successfully false otherwise.
   */
  public boolean loadRoute(IRouteLoadStrategy strategy) {
    return false;
  }
}
