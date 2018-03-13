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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneChange;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneKeeping;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AvailabilityListener;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.utils.ComponentVersion;
import std_msgs.Header;

import java.util.LinkedList;
import java.util.List;

import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

public class TrajectoryConverter {

  private final int maxPointsInPath;
  private final double timeStep;

  public TrajectoryConverter(int maxPointsInPath, double timeStep) {
    this.maxPointsInPath = maxPointsInPath;
    this.timeStep = timeStep;
  }

  public double estimateArrivalTime(double distance) {
    return 0;
  }

  public List<Point3DStamped> convertToPath(Trajectory traj, double currentTime, cav_msgs.Route route, cav_msgs.RouteState routeState) {
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
        } else {
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

  protected LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<Point3DStamped> path,
    final LongitudinalSimulationData startingData, final Route route) {

    final double startX = maneuver.getStartDistance();
    final double endX = maneuver.getEndDistance();
    final double startV = maneuver.getStartSpeed();
    final double endV = maneuver.getTargetSpeed();
    return addKinematicMotionToPath(startX, endX, startV, endV, path, startingData, route);
  }

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
   * Calculates the coefficients {a_3, a_2, a_1, a_0} of a cubic polynomial
   * 
   * The polynomial is assumed to have 0 slope at the points (x_0,y_0) and (x_1, y_1)
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
    /* Original Form before optimization
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
   * @params 
   */
  private double solveCubic(double x, double[] coefficients) {
    final double[] a = coefficients;
    final double x_sqr = x * x;
    final double x_cb = x_sqr * x;
    return a[3] * x_cb + a[2] * x_sqr + a[1] * x + a[0];
  }
}