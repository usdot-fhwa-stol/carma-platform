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
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.MessageFactory;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.LocationECEF;
import cav_msgs.LocationOffsetECEF;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Class responsible for converting Trajectories into paths described as a sequence for ECEF points for use in Mobility Messages
 * Users of this class should call the convertToPath function
 * then call the pathToMessage function to convert the path into a MobilityPath message as needed.
 * 
 * Paths are described internally as a list of ECEFPointStamped objects
 * Simple Longitudinal Maneuver motion is estimated using the basic kinematic equations of motion
 * Lateral motion is approximated using a cubic function calculated based on the start and end distance of a maneuver
 * Complex maneuvers are treated as stead speed maneuvers operating at the average of the min and max speeds of that maneuver
 * 
 * Crosstrack distance is preserved across all traversed route segments
 * Note: This class will become less reliable when dealing with lane changes along tight curves and changing lane widths
 */
public class TrajectoryConverter implements ITrajectoryConverter {

  private final int maxPointsInPath;
  private final double timeStep;
  private static final double CM_PER_M = 100.0;
  private static final double SEC_PER_MS = 0.001;
  private static final double MS_PER_SEC = 1000;
  private static final double DISTANCE_BACKWARD_TO_SEARCH = 500; //m
  private static final double DISTANCE_FORWARD_TO_SEARCH = 500; //m
  private Route route;
  private double downtrack;
  private double crosstrack;
  private int currentSegmentIdx;
  private double currentSegDowntrack;
  private int lane;

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
   * Sets the route
   * 
   * @param route The route to set
   */
  public void setRoute(Route route) {
    this.route = route;
  }

  /**
   * Gets the route
   * 
   * @return The set route
   */
  public Route getRoute() {
    return route;
  }

  /**
   * Update the segment index and current downtrack to be used for calculations
   * 
   * @param currentSegmentIdx the integer identifier of the current segment on the route
   * @param currentSegDowntrack the current progress down that segment in double-valued meters
   */
  public void setRouteState(double downtrack, double crosstrack, int currentSegmentIdx, double currentSegDowntrack, int lane) {
    this.downtrack = downtrack;
    this.crosstrack = crosstrack;
    this.currentSegmentIdx = currentSegmentIdx;
    this.currentSegDowntrack = currentSegDowntrack;
    this.lane = lane;
  }

  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj) {
    return convertToPath(traj, System.currentTimeMillis(), downtrack, crosstrack, currentSegmentIdx, currentSegDowntrack, lane);
  }

  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj, int maxPointsInPath) {
    return convertToPath(traj, System.currentTimeMillis(), downtrack, crosstrack, currentSegmentIdx, currentSegDowntrack, lane, maxPointsInPath);
  }

  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj, RoutePointStamped startPoint) {
    final long startTime = (long) ((startPoint.getStamp() + timeStep) * MS_PER_SEC);
    final double downtrack = startPoint.getDowntrack();
    final double crosstrack = startPoint.getCrosstrack();
    final int segmentIdx = startPoint.getSegmentIdx();
    final double segDowntrack = startPoint.getSegDowntrack();
    return convertToPath(traj, startTime, downtrack, crosstrack, segmentIdx, segDowntrack, lane);
  }

  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj, RoutePointStamped startPoint, int maxPointsInPath) {
    final long startTime = (long) ((startPoint.getStamp() + timeStep) * MS_PER_SEC);
    final double downtrack = startPoint.getDowntrack();
    final double crosstrack = startPoint.getCrosstrack();
    final int segmentIdx = startPoint.getSegmentIdx();
    final double segDowntrack = startPoint.getSegDowntrack();
    return convertToPath(traj, startTime, downtrack, crosstrack, segmentIdx, segDowntrack, lane, maxPointsInPath);
  }
  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj, long startTimeMS,
   double downtrack, double crosstrack,
   int currentSegmentIdx, double segDowntrack, int lane) {
    return convertToPath(traj, startTimeMS, downtrack, crosstrack, currentSegmentIdx, segDowntrack, lane, this.maxPointsInPath);
  }

  @Override
  public List<RoutePointStamped> convertToPath(Trajectory traj, long startTimeMS,
   double downtrack, double crosstrack,
   int currentSegmentIdx, double segDowntrack, int lane, int maxPointsInPath) {
    // Convert time to seconds
    final double currentTime = startTimeMS * SEC_PER_MS;
    // Get maneuvers
    List<LongitudinalManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
    List<LateralManeuver> lateralManeuvers = traj.getLateralManeuvers();
    IComplexManeuver complexManeuver = traj.getComplexManeuver();

    // Starting simulation configuration
    List<RoutePointStamped> path =  new LinkedList<RoutePointStamped>();
    final double startTime = currentTime; 
    final double startingDowntrack = downtrack;
    final double startingSegDowntrack = segDowntrack;
    final int startingSegIdx = route.getSegments().get(currentSegmentIdx).getUptrackWaypoint().getWaypointId();  
    ////
    // Process longitudinal maneuvers
    ////
    LongitudinalSimulationData longitudinalSimData = new LongitudinalSimulationData(currentTime, startingDowntrack, startingSegDowntrack, startingSegIdx);
    RoutePointStamped oldPathEndPoint = null;
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
        longitudinalSimData = addLongitudinalManeuverToPath(maneuver, path, longitudinalSimData, maxPointsInPath);
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
    double currentCrosstrack = crosstrack;
    int currentLane = lane;
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
            path.get(currentPoint).setCrosstrack(currentCrosstrack);
            currentPoint++;
          }
        } else { // A lane change will occur in this maneuver
          // Find the equation to generate the fake lane change
          double laneWidth = route.getSegments().get(path.get(currentPoint).getSegmentIdx()).getDowntrackWaypoint().getLaneWidth();
          double y_0 = currentCrosstrack;
          double y_1 = currentCrosstrack + laneWidth * -1 * maneuver.getEndingRelativeLane();
          double[] coefficients = getCubicFunction(maneuver.getStartDistance(), y_0, maneuver.getEndDistance(), y_1);
          // Apply equation to relevant points
          while (currentPoint < path.size() && maneuver.getEndDistance() > path.get(currentPoint).getDowntrack()) {
            path.get(currentPoint).setCrosstrack(solveCubic(path.get(currentPoint).getDowntrack(), coefficients));
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
      addKinematicMotionToPath(startDist, endDist, averageSpeed, averageSpeed, path, longitudinalSimData, maxPointsInPath);
    }

    return path;
  }

  @Override
  public List<ECEFPointStamped> toECEFPoints(List<RoutePointStamped> path) {
    ////
    // Convert all points to ecef frame
    ////
    List<ECEFPointStamped> ecefPoints = new ArrayList<>(path.size());
    for (RoutePointStamped point: path) {
      // Convert point to ecef
      // Currently ignores elevation
      Vector3 pointInSegmentFrame = new Vector3(point.getSegDowntrack(), point.getCrosstrack(), 0.0);
      Transform ecefToSeg =  route.getSegments().get(point.getSegmentIdx()).getECEFToSegmentTransform(); 
      Vector3 vecInECEF = ecefToSeg.apply(pointInSegmentFrame);
      // Update point
      ECEFPointStamped ecefPoint = new ECEFPointStamped();
      ecefPoint.setPoint(new Point3D(vecInECEF.getX(), vecInECEF.getY(), vecInECEF.getZ()));
      ecefPoint.setStamp(point.getStamp());
      ecefPoints.add(ecefPoint);
    }

    return ecefPoints;
  }

  @Override
  public List<RoutePointStamped> messageToPath(cav_msgs.Trajectory trajMsg) {
    return messageToPath(trajMsg, currentSegmentIdx, currentSegDowntrack);
  }
  
  @Override
  public List<RoutePointStamped> messageToPath(cav_msgs.Trajectory trajMsg, int currentSegmentIdx, double segDowntrack) {
    // Get segments within DSRC range
    List<RouteSegment> segments = route.findRouteSubsection(currentSegmentIdx, segDowntrack, DISTANCE_BACKWARD_TO_SEARCH, DISTANCE_FORWARD_TO_SEARCH);
    // Get starting location
    cav_msgs.LocationECEF startMsg = trajMsg.getLocation();
    Vector3 ecefPoint = new Vector3(startMsg.getEcefX(), startMsg.getEcefY(), startMsg.getEcefZ());
    // Get starting segment and remaining segments to search
    RouteSegment startingSegment = route.routeSegmentOfPoint(new Point3D(ecefPoint.getX(), ecefPoint.getY(), ecefPoint.getZ()), segments);
    int startIdx = startingSegment.getUptrackWaypoint().getWaypointId();

    segments = segments.subList(startIdx, segments.size() - 1);

    // Build list of route points
    List<RoutePointStamped> routePoints = new ArrayList<>(trajMsg.getOffsets().size() + 1);
    // Get starting time
    double time = startMsg.getTimestamp() / 1000L;
    // Get starting route point
    Transform ecefToSegment = startingSegment.getECEFToSegmentTransform();
    Vector3 segmentPoint = ecefToSegment.apply(new Vector3(ecefPoint.getX(), ecefPoint.getY(), ecefPoint.getZ()));
    RoutePointStamped routePoint = new RoutePointStamped(segmentPoint.getX(), segmentPoint.getY(), time);
    routePoints.add(routePoint);
    // Iterate over offsets
    for (LocationOffsetECEF offset: trajMsg.getOffsets()) {
      time += this.timeStep;
      ecefPoint = new Vector3(
        ecefPoint.getX() + offset.getOffsetX(),
        ecefPoint.getY() + offset.getOffsetY(),
        ecefPoint.getZ() + offset.getOffsetZ()
      );
      segmentPoint = ecefToSegment.apply(ecefPoint);
      
      routePoints.add(new RoutePointStamped(segmentPoint.getX(), segmentPoint.getY(), time));
    }
    
    return routePoints;
  }

  @Override
  public cav_msgs.Trajectory pathToMessage(List<RoutePointStamped> path, MessageFactory messageFactory) {
    if (path.isEmpty()) {
      return messageFactory.newFromType(cav_msgs.Trajectory._TYPE);
    }

    // Convert points to ecef
    List<ECEFPointStamped> ecefPoints = toECEFPoints(path);
    // Get message
    cav_msgs.Trajectory pathMsg = messageFactory.newFromType(cav_msgs.Trajectory._TYPE);
    // Handle starting point
    ECEFPointStamped prevPoint = ecefPoints.get(0);

    LocationECEF locationECEF = pathMsg.getLocation();
    locationECEF.setEcefX((int)(prevPoint.getPoint().getX() * CM_PER_M));
    locationECEF.setEcefY((int)(prevPoint.getPoint().getY() * CM_PER_M));
    locationECEF.setEcefZ((int)(prevPoint.getPoint().getZ() * CM_PER_M));
    locationECEF.setTimestamp((long) (prevPoint.getStamp() * MS_PER_SEC));

    // Calculate offsets
    List<LocationOffsetECEF> offsets = new ArrayList<>(ecefPoints.size()-1);
    for (int i = 1; i < ecefPoints.size(); i++) {
      ECEFPointStamped point = ecefPoints.get(i);
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

  @Override
  public LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<RoutePointStamped> path,
    final LongitudinalSimulationData startingData) {

    final double startX = maneuver.getStartDistance();
    final double endX = maneuver.getEndDistance();
    final double startV = maneuver.getStartSpeed();
    final double endV = maneuver.getTargetSpeed();
    return addKinematicMotionToPath(startX, endX, startV, endV, path, startingData);
  }

  @Override
  public LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<RoutePointStamped> path,
    final LongitudinalSimulationData startingData, final int maxPointsInPath) {

    final double startX = maneuver.getStartDistance();
    final double endX = maneuver.getEndDistance();
    final double startV = maneuver.getStartSpeed();
    final double endV = maneuver.getTargetSpeed();
    return addKinematicMotionToPath(startX, endX, startV, endV, path, startingData, maxPointsInPath);
  }

  /**
   * Helper function which generates a set of points along a route
   * which describe vehicle position based on starting and ending configurations.
   * 
   * Uses the TrajectoryConverter's current configured max path size
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
     List<RoutePointStamped> path,final LongitudinalSimulationData startingData) {
      return addKinematicMotionToPath(startX, endX, startV, endV, path, startingData, this.maxPointsInPath);
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
   * @param maxPointsInPath The maximum number of points to convert
   * 
   * @return The new configuration resulting from the motion
   */
  private LongitudinalSimulationData addKinematicMotionToPath(
    final double startX, final double endX, final double startV, final double endV,
     List<RoutePointStamped> path,final LongitudinalSimulationData startingData, int maxPointsInPath) {
      final double deltaX = endX - startX;
      final double deltaV = endV - startV;
      final double startVSqr = startV * startV;
      final double twiceAccel = (endV * endV -  startVSqr) /  deltaX;
      final double accel = twiceAccel * 0.5;
      final double deltaVPerTimeStep = accel * timeStep;

      // Ensure maxPointsInPath never exceeds the configured parameter
      maxPointsInPath = Math.min(maxPointsInPath, this.maxPointsInPath);
  
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
        if (currentSegDowntrack > currentSeg.length()) {
          currentSegDowntrack = currentSegDowntrack - currentSeg.length(); // Map distance onto new segment
          segmentIdx++;
          currentSeg = routeSegments.get(segmentIdx);
        }

        // Add point to list with timestamp
        RoutePointStamped point = new RoutePointStamped(currentDowntrack, 0.0, currentSimTime);
        point.setSegDowntrack(currentSegDowntrack);
        point.setSegmentIdx(currentSeg.getUptrackWaypoint().getWaypointId());
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