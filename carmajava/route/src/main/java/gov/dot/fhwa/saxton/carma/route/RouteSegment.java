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
import java.util.LinkedList;
import java.util.List;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.QuaternionUtils;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import org.apache.commons.lang.ArrayUtils;
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
   * Constructor initializes this segment with the given waypoints.
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
   * Gets the line segment which connects the two waypoints.
   * @return The line segment
   */
  public LineSegment3D getLineSegment() {
    return this.lineSegment;
  }

  /**
   * Gets the transform between an ECEF frame and a FRD frame located on the uptrack waypoint of this segment
   * X-Axis: Along segment
   * Y-Axis: Right of segment
   * Z-Axis: Into ground (not necessarily toward earth center if there is a change in elevation)
   */
  public Transform getECEFToSegmentTransform() {
    return this.ecefToUptrackWP;
  }

  /**
   * Helper function which calculates a FRD frame located on the uptrack waypoint of a segment.
   * X-Axis: Along segment
   * Y-Axis: Right of segment
   * Z-Axis: Into ground (not necessarily toward earth center if there is a change in elevation)
   */
  private Transform getSegmentAllignedFRDFrame() {
    Vector3D earthToUptrack = new Vector3D(this.getUptrackWaypoint().getECEFPoint());
    Vector3D earthToDowntrack = new Vector3D(this.getDowntrackWaypoint().getECEFPoint());
    Vector lineVec = this.getLineSegment().getVector(); // Vector along segment
    Vector3D newXAxis = new Vector3D(new Vector3D(lineVec.getDim(0), lineVec.getDim(1), lineVec.getDim(2)));
    Vector3D newYAxis = earthToDowntrack.cross(earthToUptrack); // This will always point to the right according to the right hand rule. I think
    Vector3D newZAxis = newXAxis.cross(newYAxis); 
    
    Vector unitXAxis = newXAxis.getUnitVector(); // Vector along segment
    Vector unitYAxis = newYAxis.getUnitVector(); // This will always point to the right according to the right hand rule. I think
    Vector unitZAxis = newZAxis.getUnitVector();

    double[][] rotMat = {
      {unitXAxis.getDim(0), unitYAxis.getDim(0), unitZAxis.getDim(0)},
      {unitXAxis.getDim(1), unitYAxis.getDim(1), unitZAxis.getDim(1)},
      {unitXAxis.getDim(2), unitYAxis.getDim(2), unitZAxis.getDim(2)}
    };

    Quaternion rotation = QuaternionUtils.matToQuaternion(rotMat);
    Vector3 translation = new Vector3(earthToUptrack.getX(), earthToUptrack.getY(), earthToUptrack.getZ());
    return new Transform(translation, rotation);
  }

  /**
   * Returns the primary lane index which matches the provided crosstrack for this segment
   * TODO: Algorithm feels overly complex and could probably be improved
   * 
   * @param crossTrack the crossTrack value to match with a lane
   * 
   * @return The lane index
   */
  public int determinePrimaryLane(double crossTrack) {
    // TODO develop a standard for this
    // Code for route going through lane center
    // int segLane = this.getDowntrackWaypoint().getLaneIndex();
    // double laneWidth = this.getDowntrackWaypoint().getLaneWidth();
    // return (int) ((double)segLane - ((crossTrack - (laneWidth / 2.0)) / laneWidth));
    //int segLane = this.getDowntrackWaypoint().getLaneIndex();

    // Code for route going through road centerline
    RouteWaypoint wp = this.getDowntrackWaypoint();
    int segLane;
    double laneWidth = wp.getLaneWidth();
    int numLanesCrossed;
    if (wp.getLaneCount() % 2 == 0) { // Even lane count
      segLane = crossTrack >= -0.0 ? wp.getLaneCount() / 2 - 1 : wp.getLaneCount() / 2;
      numLanesCrossed = (int) Math.floor((Math.abs(crossTrack)) / laneWidth);
    } else { // Odd lane count
      double edgeOffset = laneWidth / 2.0;
      segLane = wp.getLaneCount() / 2;
      if (Math.abs(crossTrack) < Math.abs(edgeOffset)) {
        return segLane;
      }
      numLanesCrossed = (int) Math.ceil((Math.abs(crossTrack) - edgeOffset) / laneWidth);
    }

    if (crossTrack >= -0.0) {
      return segLane - numLanesCrossed;
    } else {
      return segLane + numLanesCrossed;
    }
  }

  /**
   * Returns a list of lanes which are intersected by the provided bounds on the current segment
   * The returned list does not include the provided primary lane
   * 
   * @param minY the minimum y bound in segment frame
   * @param maxY the maximum y bound in segment frame
   * @param primaryLane the lane index which will not be included
   * 
   * @return A byte array of lane indices
   */
  public byte[] determineSecondaryLanes(double minY, double maxY, int primaryLane) {
    int minLane = this.determinePrimaryLane(maxY); // cross track is positive to right and negative to left
    int maxLane = this.determinePrimaryLane(minY);

    List<Byte> secondaryLanes = new LinkedList<>();
    for (int i = minLane; i <= maxLane; i++) {
      if (i != primaryLane)
        secondaryLanes.add((byte)i);
    }
    Byte[] byteObjArray = secondaryLanes.toArray(new Byte[0]);
    return ArrayUtils.toPrimitive(byteObjArray);
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
