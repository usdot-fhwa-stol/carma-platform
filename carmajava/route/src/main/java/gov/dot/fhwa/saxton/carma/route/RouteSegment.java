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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.LineSegment3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.QuaternionUtils;
import org.ros.message.MessageFactory;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.ros.rosjava_geometry.Quaternion;

/**
 * The building block of a route. Each segment is comprised of two waypoints forming a directed “vector”.
 * The first waypoint is the uptrack tail of the segment and the second waypoint is the downtrack head of the segment.
 * The properties of the second waypoint will be considered to apply for the entirety of the segment.
 */
public class RouteSegment {
  final protected RouteWaypoint uptrackWP;
  final protected RouteWaypoint downtrackWP;
  final protected LineSegment3D lineSegment;
  final protected double length;
  final protected GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
  final protected Transform ecefToUptrackWP;

  /**
   * Constructor intializes this segment with the given waypoints.
   * @param uptrackWP The uptrack waypoint for the segment to be built.
   * @param downtrackWP The downtrack waypoint for the segment to be built.
   */
  public RouteSegment(RouteWaypoint uptrackWP, RouteWaypoint downtrackWP){
    this.uptrackWP = uptrackWP;
    this.downtrackWP = downtrackWP;
    this.lineSegment = new LineSegment3D(this.uptrackWP.getECEFPoint(), this.downtrackWP.getECEFPoint());
    this.length = this.lineSegment.length();
    this.ecefToUptrackWP = getSegmentAllignedFRDFrame();
  }

  /**
   * Calculates the crosstrack distance from the provided GPS location to this route segment
   * Uses flat earth model
   * 
   * @param location The gps location to be compared
   * @return The calculated cross track distance in meters
   */
  public double crossTrackDistance(Location location) {
    Point3D ecefPoint = gcc.geodesic2Cartesian(location, Transform.identity());
    return crossTrackDistance(ecefPoint);
  }

  /**
   * Calculates the downtrack distance from the provided GPS location to this route segment start
   * Uses flat earth model
   *
   * @param location The gps location to be compared
   * @return The calculated down track distance in meters
   */
  public double downTrackDistance(Location location) {
    Point3D ecefPoint = gcc.geodesic2Cartesian(location, Transform.identity());
    return downTrackDistance(ecefPoint);
  }

  /**
   * Calculates location of a external point projected onto the segment
   * Uses flat earth model
   *
   * @param loc The location whose projection is being calculated
   * @return The projected location
   */
  public Location projectOntoSegment(Location location) {
    Point3D ecefPoint = gcc.geodesic2Cartesian(location, Transform.identity());
    return gcc.cartesian2Geodesic(projectOntoSegment(ecefPoint), Transform.identity());
  }

    /**
   * Calculates the crosstrack distance from the provided GPS location to this route segment
   * Uses flat earth model
   *
   * @param point The gps location to be compared
   * @return The calculated cross track distance in meters
   */
  public double crossTrackDistance(Point3D point) {
    return this.lineSegment.crossTrackDistance(point);
  }

  /**
   * Calculates the downtrack distance from the provided GPS location to this route segment start
   * Uses flat earth model
   *
   * @param point The gps location to be compared
   * @return The calculated down track distance in meters
   */
  public double downTrackDistance(Point3D point) {
    return this.lineSegment.downtrackDistance(point);
  }

  /**
   * Calculates location of a external point projected onto the segment
   * Uses flat earth model
   *
   * @param point The location whose projection is being calculated
   * @return The projected location
   */
  public Point3D projectOntoSegment(Point3D point) {
    return this.lineSegment.projectOntoSegment(point);
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
   * Gets a deep copy of the line segment which connects the two waypoints.
   * @return The line segment
   */
  public LineSegment3D getLineSegment() {
    return new LineSegment3D(this.lineSegment);
  }

  /**
   * Gets the transform between an ECEF frame and a FRD frame located on the uptrack waypoint of this segment
   * X-Axis: Along segment
   * Y-Axis: Right of segment
   * Z-Axis: Into ground (not nessisarily toward earch center if there is a change in elevation)
   */
  public Transform getECEFToSegmentTransform() {
    return this.ecefToUptrackWP;
  }

  /**
   * Helper function which calculates a FRD frame located on the uptrack waypoint of a segment.
   * X-Axis: Along segment
   * Y-Axis: Right of segment
   * Z-Axis: Into ground (not nessisarily toward earch center if there is a change in elevation)
   */
  private Transform getSegmentAllignedFRDFrame() {
    Vector3D earthToUptrack = new Vector3D(this.getUptrackWaypoint().getECEFPoint());
    Vector3D earthToDowntrack = new Vector3D(this.getDowntrackWaypoint().getECEFPoint());
    Vector3D newXAxis = (Vector3D) this.getLineSegment().getVector().getUnitVector(); // Vector along segment
    Vector3D newYAxis = (Vector3D) earthToDowntrack.cross(earthToUptrack).getUnitVector(); // This will always point to the right according to the right hand rule. I think
    Vector3D newZAxis = (Vector3D) newXAxis.cross(newYAxis).getUnitVector(); 

    double[][] rotMat = {
      {newXAxis.getX(), newYAxis.getX(), newZAxis.getX()},
      {newXAxis.getY(), newYAxis.getY(), newZAxis.getY()},
      {newXAxis.getZ(), newYAxis.getZ(), newZAxis.getZ()}
    };

    Quaternion rotation = QuaternionUtils.matToQuaternion(rotMat);
    Vector3 translation = new Vector3(earthToUptrack.getX(), earthToUptrack.getY(), earthToUptrack.getZ());
    return new Transform(translation, rotation);
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
