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

package gov.dot.fhwa.saxton.carma.guidance.util;

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
  public double estimateArrivalTime(double distance) {
    return 0;
  }

  public List<Point3DStamped> convertToPath(Trajectory traj) {

    List<LongitudinalManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
    List<LateralManeuver> lateralManeuvers = traj.getLateralManeuvers();
    Point3DStamped startPoint = new Point3DStamped();
    startPoint.setPoint(new Point3D(0, 0, 0)); // TODO use real ecef location
    startPoint.setStamp(System.currentTimeMillis()); // TODO use real GPS time
    List<Point3DStamped> path =  new LinkedList<Point3DStamped>();
    path.add(startPoint);
    // Process longitudinal maneuvers
    for (LongitudinalManeuver maneuver: longitudinalManeuvers) {
      maneuver.getStartDistance();
    }
    return new LinkedList<Point3DStamped>();
  }

  private List<Point3DStamped> longitudinalManeuverToPoints(
    final LongitudinalManeuver maneuver, final double startTime, final double timeStep, 
    RouteSegment currentSeg, final RouteState routeState, final Route route) {

    // TODO
    // I need to add the segment frame into my route messages even though it will break my rosbags
    // Look at fix/roadway_obstacles for an example of how to do this (might have branched off that I can't remember)
    final double startV = maneuver.getStartSpeed();
    final double deltaX = maneuver.getEndDistance() - maneuver.getStartDistance();
    final double deltaV = maneuver.getTargetSpeed() - startV;
    final double deltaT = deltaX / deltaV;
    final double accel = deltaV / deltaT;
    final double accelTerm = 0.5 * timeStep * timeStep;
    double currentSimTime = 0;
    List<Point3DStamped> points = new LinkedList<Point3DStamped>();
    List<RouteSegment> routeSegments = route.getSegments();

    double startX = routeState.getSegmentDownTrack(); // Start measurement from current segment downtrack
    int segmentIdx = currentSeg.getPrevWaypoint().getWaypointId();

    while(currentSimTime < startTime + deltaT) {
      double newDist = startX + accelTerm + startV * timeStep;

      // If past the current segment get the next one
      if (newDist > currentSeg.getLength()) {
        newDist = newDist - currentSeg.getLength(); // Map distance onto new segment
        segmentIdx++;
        currentSeg = routeSegments.get(segmentIdx);
      }
      // Convert point to ecef
      Vector3 pointInSegmentFrame = new Vector3(newDist, 0, 0);
      Transform ecefToSeg = Transform.fromTransformMessage(currentSeg.getECEFToSegTransform()); // TODO If the route was a java object this conversion might not be needed
      Vector3 vecInECEF = ecefToSeg.apply(pointInSegmentFrame);
      // Add point to list with timestamp
      Point3DStamped point = new Point3DStamped();
      point.setPoint(new Point3D(vecInECEF.getX(), vecInECEF.getY(), vecInECEF.getZ()));
      point.setStamp(currentSimTime);
      points.add(point);
      // Update starting distance, speed, and current time
      startX = newDist;
      startV += accel * timeStep;
      currentSimTime += timeStep;
    }

    return points;
  }

  private class Point3DStamped {
    Point3D point;
    double stamp;

    public void setPoint(Point3D point){
      this.point = point;
    }

    public void setStamp(double stamp){
      this.stamp = stamp;
    }

    public Point3D getPoint(){
      return point;
    }

    public double getStamp(){
      return stamp;
    }
  }
}