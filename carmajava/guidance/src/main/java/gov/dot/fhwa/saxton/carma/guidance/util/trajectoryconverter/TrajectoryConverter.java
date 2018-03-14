/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.FutureLongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageFactory;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import cav_msgs.MobilityPath;
import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Class responsible for converting Trajectories into paths described as a sequence for ECEF points for use in Mobility Messages
 * Users of this class should call the convertToPath function
 * then call the pathToMessage function to convert the path into a MobilityPath message as needed.
 * 
 * Paths are described internally as a list of Point3DStamped objects
 * Simple Longitudinal Maneuver motion is estimated using the basic kinematic equations of motion
 * Lateral motion is approximated using a cubic function calculated based on the start and end distance of a maneuver
 * Complex maneuvers are treated as stead speed maneuvers operating at the average of the min and max speeds of that maneuver
 * 
 * Crosstrack distance is preserved across all traversed route segments
 * Note: This class will become less reliable when dealing with lane changes along tight curves and changing lane widths
 */
public class TrajectoryConverter {

  private final int maxPointsInPath;
  private final double timeStep;
  final double CM_PER_M = 100.0;
  final double SEC_PER_MS = 0.001;
  final double MS_PER_SEC = 1000;

  /**
   * Constructor
   * 
   * @param maxPointsInPath The maximum number of points which will be included in a path
   * @param timeStep The size in seconds of the time step separating each point in a path. 
   */
  public TrajectoryConverter(int maxPointsInPath, double timeStep) {
    this.maxPointsInPath = maxPointsInPath;
    this.timeStep = timeStep;
  }

  /**
   * Converts the provided trajectory and starting configuration into a list of ecef points with associated time stamps
   * 
   * This function determines all the point downtrack distances using simple longitudinal maneuvers and kinematic equations.
   * Then the longitudinal maneuvers are used to shift the crosstrack values of each point
   * Then any complex maneuvers are added to the path
   * Finally all points are converted into the ECEF frame
   * 
   * @param traj The trajectory to convert
   * @param currentTimeMs The starting time for this path in ms 
   * @param route A route message containing all possible route segments
   * @param routeState A route state message containing the current segment and progress along the route
   * 
   * @return A list of ecef points associated with time stamps and segments
   */
  public List<Point3DStamped> convertToPath(Trajectory traj, long startTimeMS, cav_msgs.Route route, cav_msgs.RouteState routeState) {
    // Convert time to seconds
    final double currentTime = startTimeMS * SEC_PER_MS;
    // Get maneuvers
    List<LongitudinalManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
    List<LateralManeuver> lateralManeuvers = traj.getLateralManeuvers();
    IComplexManeuver complexManeuver = traj.getComplexManeuver();

    // Starting simulation configuration
    List<Point3DStamped> path =  new LinkedList<Point3DStamped>();
    final double startTime = currentTime; 
    final double startingDowntrack = routeState.getDownTrack();
    final double startingSegDowntrack = routeState.getSegmentDownTrack();
    final int startingSegIdx = routeState.getCurrentSegment().getPrevWaypoint().getWaypointId();  
    ////
    // Process longitudinal maneuvers
    ////
    LongitudinalSimulationData longitudinalSimData = new LongitudinalSimulationData(currentTime, startingDowntrack, startingSegDowntrack, startingSegIdx);
    Point3DStamped oldPathEndPoint = null;
    int oldPathSize = 0;
    for (int i = 0; i < longitudinalManeuvers.size(); i++) {
      LongitudinalManeuver maneuver = longitudinalManeuvers.get(i);
      // If this is a future maneuver which has been filled with maneuvers we will replace it with those maneuvers
      if (maneuver instanceof FutureLongitudinalManeuver && !((FutureLongitudinalManeuver)maneuver).getLongitudinalManeuvers().isEmpty()) {
        longitudinalManeuvers.remove(i);
        longitudinalManeuvers.addAll(i, ((FutureLongitudinalManeuver) maneuver).getLongitudinalManeuvers());
        maneuver = longitudinalManeuvers.get(i);
      }

      // If this maneuver is happening or will happen add it to the path
      if (maneuver.getEndDistance() > longitudinalSimData.downtrack) {
        longitudinalSimData = addLongitudinalManeuverToPath(maneuver, path, longitudinalSimData, route);
        // Ensure there are no overlapping points in time
        if (oldPathEndPoint != null && oldPathEndPoint.getStamp() == path.get(oldPathSize).getStamp()){
          path.remove(oldPathSize);
        }
        oldPathSize = path.size();
        oldPathEndPoint = path.get(path.size() - 1);
      }
    }
    ////
    // Process Lateral Maneuvers
    ////
    int currentPoint = 0;
    double currentCrosstrack = routeState.getCrossTrack();
    int currentLane = routeState.getLaneIndex();
    for (int i = 0; i < lateralManeuvers.size(); i++) {
      if (currentPoint >= path.size()) {
        break;
      }
      LateralManeuver maneuver = lateralManeuvers.get(i);
      // If this is a future maneuver which has been filled with maneuvers we will replace it with those maneuvers
      if (maneuver instanceof FutureLateralManeuver && !((FutureLateralManeuver)maneuver).getLateralManeuvers().isEmpty()) {
        lateralManeuvers.remove(i);
        lateralManeuvers.addAll(i, ((FutureLateralManeuver) maneuver).getLateralManeuvers());
        maneuver = lateralManeuvers.get(i);
      }
      // If this maneuver is happening or will happen add it to the path
      if (maneuver.getEndDistance() > path.get(currentPoint).getDowntrack()) {
        // If no lane change occurs we will maintain the current crosstrack
        if (maneuver.getEndingRelativeLane() == 0) {
          while (currentPoint < path.size() && maneuver.getEndDistance() > path.get(currentPoint).getDowntrack()) {
            path.get(currentPoint).getPoint().setY(currentCrosstrack);
            currentPoint++;
          }
        } else { // A lane change will occur in this maneuver
          // Find the equation to generate the fake lane change
          double laneWidth = route.getSegments().get(path.get(currentPoint).getSegmentIdx()).getWaypoint().getLaneWidth();
          double y_0 = currentCrosstrack;
          double y_1 = currentCrosstrack + laneWidth * -1 * maneuver.getEndingRelativeLane();
          double[] coefficients = getCubicFunction(maneuver.getStartDistance(), y_0, maneuver.getEndDistance(), y_1);
          // Apply equation to relevant points
          while (currentPoint < path.size() && maneuver.getEndDistance() > path.get(currentPoint).getDowntrack()) {
            path.get(currentPoint).getPoint().setY(solveCubic(path.get(currentPoint).getDowntrack(), coefficients));
            currentPoint++;
          }
          currentCrosstrack = y_1;
          currentLane += maneuver.getEndingRelativeLane();
        }
      }
    }

    ////
    // Process Complex Maneuvers
    ////
    // This is a very simplistic handling of the complex maneuver.
    // If complex maneuvers become more common this should be changed
    if (complexManeuver != null) {
      double averageSpeed = 0.5 * (complexManeuver.getMinExpectedSpeed() + complexManeuver.getMaxExpectedSpeed());
      double startDist = complexManeuver.getStartDistance();
      double endDist = complexManeuver.getEndDistance();
      // Treat complex maneuver as stead speed maneuver
      addKinematicMotionToPath(startDist, endDist, averageSpeed, averageSpeed, path, longitudinalSimData, route);
    }

    ////
    // Convert all points to ecef frame
    ////
    for (Point3DStamped point: path) {
      // Convert point to ecef
      // Currently ignores elevation
      Vector3 pointInSegmentFrame = new Vector3(point.getPoint().getX(), point.getPoint().getY(), 0);
      Transform ecefToSeg = Transform.fromPoseMessage(route.getSegments().get(point.getSegmentIdx()).getFRDPose()); 
      Vector3 vecInECEF = ecefToSeg.apply(pointInSegmentFrame);
      // Update point
      point.setPoint(new Point3D(vecInECEF.getX(), vecInECEF.getY(), vecInECEF.getZ()));
    }

    return path;
  }

  /**
   * Function converts a path to a MobilityPath message using the provided message factory
   * 
   * @param path The list of ecef points and times which defines the path
   * @param messageFactory The message factory which will be used to build this message
   * 
   * @return A MobilityPath message. This message will be empty if the path was empty
   */
  public MobilityPath pathToMessage(List<Point3DStamped> path, MessageFactory messageFactory) {
    if (path.isEmpty()) {
      return messageFactory.newFromType(MobilityPath._TYPE);
    }
    MobilityPath pathMsg = messageFactory.newFromType(MobilityPath._TYPE);
    // Handle starting point
    Point3DStamped prevPoint = path.get(0);

    LocationECEF locationECEF = pathMsg.getLocation();
    locationECEF.setEcefX((int)(prevPoint.getPoint().getX() * CM_PER_M));
    locationECEF.setEcefY((int)(prevPoint.getPoint().getY() * CM_PER_M));
    locationECEF.setEcefZ((int)(prevPoint.getPoint().getZ() * CM_PER_M));
    locationECEF.setTimestamp((long) (prevPoint.getStamp() * MS_PER_SEC));

    // Calculate offsets
    List<LocationOffsetECEF> offsets = new ArrayList<>(path.size()-1);
    for (int i = 1; i < path.size(); i++) {
      Point3DStamped point = path.get(i);
      LocationOffsetECEF offsetMsg = messageFactory.newFromType(LocationOffsetECEF._TYPE);
      double deltaX = point.getPoint().getX() - prevPoint.getPoint().getX();
      double deltaY = point.getPoint().getY() - prevPoint.getPoint().getY();
      double deltaZ = point.getPoint().getZ() - prevPoint.getPoint().getZ();
      offsetMsg.setOffsetX((short)(deltaX * CM_PER_M));
      offsetMsg.setOffsetY((short)(deltaY * CM_PER_M));
      offsetMsg.setOffsetZ((short)(deltaZ * CM_PER_M));
      offsets.add(offsetMsg);
      prevPoint = point;
    }
    pathMsg.setOffsets(offsets);
    return pathMsg;
  }

  /**
   * Function which converts and individual Simple Longitudinal Maneuver to a path based on starting configuration
   * 
   * @param maneuver The maneuver to convert
   * @param path The list which will store the generated points
   * @param startingData The starting configuration of the vehicle
   * @param route The route the vehicle is on
   */
  public LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<Point3DStamped> path,
    final LongitudinalSimulationData startingData, final Route route) {

    final double startX = maneuver.getStartDistance();
    final double endX = maneuver.getEndDistance();
    final double startV = maneuver.getStartSpeed();
    final double endV = maneuver.getTargetSpeed();
    return addKinematicMotionToPath(startX, endX, startV, endV, path, startingData, route);
  }

  /**
   * Helper function which generates a set of points along a route
   * which describe vehicle position based on starting and ending configurations.
   * 
   * @param startX The starting downtrack location on the route
   * @param endX The ending downtrack location on the route
   * @param startV The starting velocity along the route
   * @param endV The ending velocity along the route
   * @param path The list of points which will be added to
   * @param startingData The starting configuration
   * @param route The route being traversed
   * 
   * @return The new configuration resulting from the motion
   */
  private LongitudinalSimulationData addKinematicMotionToPath(
    final double startX, final double endX, final double startV, final double endV,
     List<Point3DStamped> path,final LongitudinalSimulationData startingData, final Route route) {
      final double deltaX = endX - startX;
      final double deltaV = endV - startV;
      final double startVSqr = startV * startV;
      final double twiceAccel = (endV * endV -  startVSqr) /  deltaX;
      final double accel = twiceAccel * 0.5;
      final double deltaVPerTimeStep = accel * timeStep;
  
      // If this is the current maneuver only use the remaining distance
      double actualStartV;
      final double actualDeltaV, actualDeltaX;
      if (startX < startingData.downtrack) {
        actualDeltaX = endX - startingData.downtrack;
        actualStartV = Math.sqrt(startVSqr + twiceAccel * actualDeltaX);
        actualDeltaV = actualStartV - startV;
      } else {
        actualStartV = startV;
        actualDeltaV = deltaV;
        actualDeltaX = deltaX;
      }
  
      final double deltaT;
      if (accel == 0.0) {
        deltaT = actualDeltaX / actualStartV;
      } else {
        deltaT = actualDeltaV / accel;
      }
       
  
      final double accelTerm = 0.5 * accel * timeStep * timeStep;
      final double endTime = startingData.simTime + deltaT;
      double currentSimTime = startingData.simTime;
      final List<RouteSegment> routeSegments = route.getSegments();
  
      RouteSegment currentSeg = routeSegments.get(startingData.segmentIdx);
      int segmentIdx = startingData.segmentIdx;
  
      double currentSegDowntrack = startingData.segmentDowntrack;
      double currentDowntrack = startingData.downtrack;
      double distanceChange = 0;
      while(currentSimTime <= endTime && path.size() <= maxPointsInPath) {
        // If past the current segment get the next one
        if (currentSegDowntrack > currentSeg.getLength()) {
          currentSegDowntrack = currentSegDowntrack - currentSeg.getLength(); // Map distance onto new segment
          segmentIdx++;
          currentSeg = routeSegments.get(segmentIdx);
        }

        // Add point to list with timestamp
        Point3DStamped point = new Point3DStamped();
        point.setPoint(new Point3D(currentSegDowntrack, 0, 0));
        point.setStamp(currentSimTime);
        point.setSegmentIdx(segmentIdx);
        point.setDowntrack(currentDowntrack);
        path.add(point);
        // Update starting distance, speed, and current time
        distanceChange = accelTerm + actualStartV * timeStep;
        currentDowntrack += distanceChange;
        currentSegDowntrack += distanceChange; 
        actualStartV += deltaVPerTimeStep;
        currentSimTime += timeStep;
      }
  
      return new LongitudinalSimulationData(currentSimTime - timeStep, currentDowntrack - distanceChange, currentSegDowntrack - distanceChange, segmentIdx);  
     }

  /**
   * Calculates the coefficients {a_0, a_1, a_2, a_3} of a cubic polynomial
   * 
   * The polynomial is assumed to have 0 slope at the points (x_0, y_0) and (x_1, y_1)
   * 
   * The equations used to generate the solution are as follows.
   * These were symbolically reduced to equations for the constants using an equation solver. 
   * 
   * y_0 = a_3 x_0^3 + a_2 x_0^2 + a_1 x_0 + a_0
   * y_1 = a_3 x_1^3 + a_2 x_1^2 + a_1 x_1 + a_0
   * 0 = 3 a_3 x_0^2 + 2 a_2 x_0 + a_1
   * 0 = 3 a_3 x_1^2 + 2 a_2 x_1 + a_1
   * 
   * 
   * Note: This is only valid for x_0 != x_1 and y_0 != y1
   * 
   * @param x_0 The first x value
   * @param y_0 The first y value
   * @param x_1 The second x value
   * @param y_1 The second y value
   * 
   * @return An array of coefficients of the form {a_0, a_1, a_2, a_3}
   * 
   */
  private double[] getCubicFunction(double x_0, double y_0, double x_1, double y_1) {
    // When x_0 != x_1 and y_0 != y_1
    final double x_0_sqr = x_0 * x_0;
    final double x_0_cb = x_0_sqr * x_0;
    final double x_1_sqr = x_1 * x_1;
    final double x_1_cb = x_1_sqr * x_1;
    
    final double invDeltaX = x_0 - x_1;
    final double invDeltaXCubed = invDeltaX * invDeltaX * invDeltaX;

    final double invDeltaY = y_0 - y_1;
    /* Original Form before optimizations
    a_3 = -(2 * (y_0 - y_1))/(x_0 - x_1)^3;
    a_2 = (3 * (x_0 + x_1) * (y_0 - y_1))/(x_0 - x_1)^3;
    a_1 = (6 * x_0 *x_1 * (y_1 - y_0))/(x_0 - x_1)^3;
    a_0 = (x_0^3 * y_1 - 3 * x_1 * x_0^2 * y_1 + 3 * x_1^2 * x_0 * y_0 - x_1^3 * y_0)/(x_0 - x_1)^3
     */
    final double a_3 = -(2 * (invDeltaY))/invDeltaXCubed;
    final double a_2 = (3 * (x_0 + x_1) * (invDeltaY))/invDeltaXCubed;
    final double a_1 = (6 * x_0 * x_1 * (y_1 - y_0))/invDeltaXCubed;
    final double a_0 = (x_0_cb * y_1 - 3 * x_1 * x_0_sqr * y_1 + 3 * x_1_sqr * x_0 * y_0 - x_1_cb * y_0)/invDeltaXCubed;

    final double[] coefficients = {a_0, a_1, a_2, a_3};
    return coefficients;
  }

   /**
   * Solves a cubic polynomial with the provided coefficients
   * f(x) = a_3 * x^3 + a_2 * x^2 + a_1 * x + a_0
   * 
   * @param x The value to solve for
   * @param coefficients An array of coefficients which define the cubic equation must be of the form {a_0, a_1, a_2, a_3}
   */
  private double solveCubic(double x, double[] coefficients) {
    final double[] a = coefficients;
    final double x_sqr = x * x;
    final double x_cb = x_sqr * x;
    return a[3] * x_cb + a[2] * x_sqr + a[1] * x + a[0];
  }
}