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

import gov.dot.fhwa.saxton.carma.geometry.geodesic.EarthSegment;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.ros.message.MessageFactory;

/**
 * The building block of a route. Each segment is comprised of two waypoints forming a directed “vector”.
 * The first waypoint is the uptrack tail of the segment and the second waypoint is the downtrack head of the segment.
 * The properties of the second waypoint will be considered to apply for the entirety of the segment.
 */
public class RouteSegment {
  final protected RouteWaypoint uptrackWP;
  final protected RouteWaypoint downtrackWP;
  final protected EarthSegment earthSegment;
  final protected double length;

  /**
   * Constructor intializes this segment with the given waypoints.
   * @param uptrackWP The uptrack waypoint for the segment to be built.
   * @param downtrackWP The downtrack waypoint for the segment to be built.
   */
  public RouteSegment(RouteWaypoint uptrackWP, RouteWaypoint downtrackWP){
    this.uptrackWP = uptrackWP;
    this.downtrackWP = downtrackWP;
    this.earthSegment = new EarthSegment(this.uptrackWP.getLocation(), this.downtrackWP.getLocation());
    length = 0; // TODO: Calculate the length here
  }

  /**
   * Calculates the crosstrack distance from the provided GPS location to this route segment
   * TODO implement
   *
   * @param location The gps location to be compared
   * @return The calculated cross track distance in meters
   */
  public double crossTrackDistance(Location location) {
    return 0;
  }

  /**
   * Returns the length of this segment in meters
   * @return The length of the segment in meters
   */
  public double length(){
    return this.length;
  }

  /**
   * Gets the uptrack waypoint of this segment
   * @return The uptrack waypoint
   */
  public RouteWaypoint getUptrackWaypoint(){
    return uptrackWP;
  }

  /**
   * Gets the downtrack waypoint of this segment
   * @return The downtrack waypoint of this segment
   */
  public RouteWaypoint getDowntrackWaypoint(){
    return downtrackWP;
  }

  /**
   * Constructs a fully initialized ros message from this route segment
   * @param factory The message factory which will be used to get a ros message object
   * @return The ros message
   */
  public cav_msgs.RouteSegment toMessage(MessageFactory factory, int downtrackWPIndex){
    cav_msgs.RouteSegment routeSegMsg = factory.newFromType(cav_msgs.RouteSegment._TYPE);
    routeSegMsg.setLength(length);
    routeSegMsg.setPrevWaypoint(uptrackWP.toMessage(factory, downtrackWPIndex - 1));
    routeSegMsg.setWaypoint(downtrackWP.toMessage(factory, downtrackWPIndex));

    return routeSegMsg;
  }

  /**
   * Converts a ros message into an initialized RouteSegment object
   * @param segmentMsg The ros message
   * @return The route segment object
   */
  public static RouteSegment fromMessage(cav_msgs.RouteSegment segmentMsg){
    return new RouteSegment(RouteWaypoint.fromMessage(segmentMsg.getPrevWaypoint()),RouteWaypoint.fromMessage(segmentMsg.getWaypoint()));
  }
}
