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

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
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

    // Process longitudinal maneuvers
    LongitudinalSimulationData longitudinalSimData = new LongitudinalSimulationData(currentTime, startingDowntrack, startingSegDowntrack, startingSegIdx);
    Point3DStamped oldPathEndPoint = null;
    int oldPathSize = 0;
    for (LongitudinalManeuver maneuver: longitudinalManeuvers) {
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

    // TODO process lateral maneuvers
    // TODO process complex maneuver
    return path;
  }

  protected LongitudinalSimulationData addLongitudinalManeuverToPath(
    final LongitudinalManeuver maneuver, List<Point3DStamped> path,
    final LongitudinalSimulationData startingData, final Route route) {

    final double startX = maneuver.getStartDistance();
    final double endX = maneuver.getEndDistance();
    final double startV = maneuver.getStartSpeed();
    final double endV = maneuver.getTargetSpeed();
    final double deltaX = endX - startX;
    final double deltaV = endV - startV;
    final double startVSqr = startV * startV;
    final double twiceAccel = (endV * endV -  startVSqr) /  deltaX;
    final double accel = twiceAccel * 0.5;
    final double deltaVPerTimeStep = accel * timeStep;

    // If this is the current maneuver only use the remaining distance
    double actualStartV;
    final double actualDeltaV, actualDeltaX;
    if (maneuver.getStartDistance() < startingData.downtrack) {
      actualDeltaX = maneuver.getEndDistance() - startingData.downtrack;
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
      // Convert point to ecef
      Vector3 pointInSegmentFrame = new Vector3(currentSegDowntrack, 0, 0);
      Transform ecefToSeg = Transform.fromPoseMessage(currentSeg.getFRDPose()); 
      Vector3 vecInECEF = ecefToSeg.apply(pointInSegmentFrame);
      // Add point to list with timestamp
      Point3DStamped point = new Point3DStamped();
      point.setPoint(new Point3D(vecInECEF.getX(), vecInECEF.getY(), vecInECEF.getZ()));
      point.setStamp(currentSimTime);
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
}