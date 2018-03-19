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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.AxisAlignedBoundingBox;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap.NSpatialHashMap;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap.SimpleHashStrategy;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.TrajectoryConverter;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import cav_msgs.MobilityPath;

/**
 * An implementation of an OcTree for use in the Conflict Detector
 * 
 * Used to evaluate trajectories and mobility paths for conflicts
 */
public class ConflictDetector implements IConflictManager {
  // Axis number of route values in a point on the tree
  private static final int DOWNTRACK_IDX = 0;
  private static final int CROSSTRACK_IDX = 1;
  private static final int TIME_IDX = 2;
  // The maximum size of a cell in the tree
  private double[] maxSize = {5,5,0.1};

  List<NSpatialHashMap> spatialMaps = new LinkedList<>();
  private NSpatialHashMap spatialMap = new NSpatialHashMap(new AxisAlignedBoundingBox(), new SimpleHashStrategy(maxSize));

  /**
   * Constructor
   */
  public ConflictDetector() {}

  @Override
  public void addPath(MobilityPath pathMsg) {
    String senderStaticID = pathMsg.getHeader().getSenderId();
    String planID = pathMsg.getHeader().getPlanId(); // TODO decide if needed
  }


  @Override
  public void removePath(MobilityPath pathMsg) {
    
  }


  @Override // TODO add support for multiple maps
  public List<ConflictSpace> getConflicts(Trajectory traj, long startTime,
   cav_msgs.Route route, cav_msgs.RouteState routeState) {
    
    TrajectoryConverter tc = new TrajectoryConverter(60, 0.1); // TODO
    List<RoutePointStamped> points = tc.convertToPath(traj, startTime, route, routeState);
    List<Point3D> trajPoints = new ArrayList<>(points.size());
    // Convert points into (downtrack, crosstrack, time);
    for (RoutePointStamped routePoint: points) {
      trajPoints.add(new Point3D(routePoint.getPoint().getY(), routePoint.getDowntrack(), routePoint.getStamp()));
    }

    List<ConflictSpace> conflicts = new LinkedList<>();
    ConflictSpace currentConflict = null;
    int lane = 0; // TODO lane for each point
    Point prevPoint = null;
    for (Point3D p: trajPoints) {
      // Get conflicts with point
      if (!spatialMap.getCollisions(p).isEmpty()) { 
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(p.getDim(DOWNTRACK_IDX), p.getDim(TIME_IDX), lane);
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
          currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDim(DOWNTRACK_IDX), prevPoint.getDim(TIME_IDX), lane);
        } 
      } else {
        // If we could insert the point but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
          currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = p;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
      currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
      conflicts.add(currentConflict);
    }

    return conflicts;
  }
}