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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap.NSpatialHashMap;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap.NSpatialHashMapFactory;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialhashmap.SimpleHashStrategy;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.TrajectoryConverter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import cav_msgs.MobilityPath;
import cav_msgs.Route;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

/**
 * An implementation of an OcTree for use in the Conflict Detector
 * 
 * Used to evaluate trajectories and mobility paths for conflicts
 */
public class ConflictDetector implements IConflictManager {
  // The maximum size of a cell in the tree
  private double[] maxSize = {5,5,0.1};
  private Vector3D standardSizeVec = new Vector3D(2.5, 1.1, 1.1);
  private double timeStep = 0.1;

  private Map<String, NSpatialHashMap> mobilityPathSpatialMaps = new HashMap<>();
  private Map<String, NSpatialHashMap> requestedPathSpatialMaps = new HashMap<>();

  /**
   * Constructor
   */
  public ConflictDetector() {}

  @Override
  public boolean addMobilityPath(List<RoutePointStamped> path, String vehicleStaticId) {
    return addPath(path, vehicleStaticId, mobilityPathSpatialMaps);
  }

  @Override
  public boolean addRequestedPath(List<RoutePointStamped> path, String planId) {
    return addPath(path, planId, requestedPathSpatialMaps);
  }
  
  /**
   * Helper function fro adding paths to a map of spatial hash maps
   * 
   * @param path The path to add for future conflict detection
   * @param key The key to use for identifying this path
   * 
   * @return True if the path could be added. False if not.
   */
  private boolean addPath(List<RoutePointStamped> path, String key, Map<String, NSpatialHashMap> map) {
    // Get current path for this vehicle
    NSpatialHashMap vehiclesPath = map.get(key);
    // If not current path for this vehicle add it 
    if (vehiclesPath == null) {
      vehiclesPath =  NSpatialHashMapFactory.buildSpatialHashMap(maxSize);
      map.put(key, vehiclesPath);
    }
    // Insert points
    return insertPoints(path, vehiclesPath);
  }

  /**
   * Helper function inserts a list of RoutePointStamped into a NSpatialHashMap
   * 
   * @param path The points to insert
   * @param map The map to insert points into
   * 
   * @return True if the path could be added. False if not.
   */
  private boolean insertPoints(List<RoutePointStamped> path, NSpatialHashMap map) {
    // Add points to spatial map
    for (RoutePointStamped routePoint: path) {
      // Define bounds
      Point3D minBoundingPoint = new Point3D(
        routePoint.getDowntrack() - standardSizeVec.getX(),
        routePoint.getCrosstrack() - standardSizeVec.getY(),
        routePoint.getStamp() - timeStep);
      Point3D maxBoundingPoint = new Point3D(
        routePoint.getDowntrack() + standardSizeVec.getX(),
        routePoint.getCrosstrack() + standardSizeVec.getY(),
        routePoint.getStamp() + timeStep);
      // Insert point
      map.insert(new CartesianObject(Arrays.asList(minBoundingPoint, maxBoundingPoint)));
    }
    return true;
  }
  
  @Override
  public boolean removeMobilityPath(String vehicleStaticId) {
    return mobilityPathSpatialMaps.remove(vehicleStaticId) != null ? true :  false;
  }

  @Override
  public boolean removeRequestedPath(String planId) {
    return requestedPathSpatialMaps.remove(planId) != null ? true :  false;
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath) {
    // Prepare to store conflicts
    List<ConflictSpace> conflicts = new LinkedList<>();
    // Get possible collision sets
    Collection<NSpatialHashMap> mobilityPaths = mobilityPathSpatialMaps.values();
    Collection<NSpatialHashMap> requestedPaths = requestedPathSpatialMaps.values();

    // Iterate over all points in the host path
    ConflictSpace currentConflict = null;
    int lane = 0;
    RoutePointStamped prevPoint = null;
    
    for (RoutePointStamped routePoint: hostPath) {
      // Get lane
      lane = RouteSegment.fromMessage(routePoint.getSegment()).determinePrimaryLane(routePoint.getDowntrack());

      // Check for collisions
      boolean hasCollision = false;
      // Check mobility paths
      for (NSpatialHashMap map: mobilityPaths) {
        if (map.surrounds(routePoint.getPoint())) {
          if (!map.getCollisions(routePoint.getPoint()).isEmpty()) {
            hasCollision = true;
            break;
          }
        }
      }
      if (!hasCollision) {
        // Check requested paths
        for (NSpatialHashMap map: requestedPaths) {
          if (map.surrounds(routePoint.getPoint())) {
            if (!map.getCollisions(routePoint.getPoint()).isEmpty()) {
              hasCollision = true;
              break;
            }
          }
        }
      }
      // Update conflicts
      if (hasCollision) { 
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane, routePoint.getSegment());
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          currentConflict.setEndDowntrack(prevPoint.getDowntrack());
          currentConflict.setEndTime(prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane, routePoint.getSegment());
        } 
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          currentConflict.setEndDowntrack(prevPoint.getDowntrack());
          currentConflict.setEndTime(prevPoint.getStamp());
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = routePoint;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      currentConflict.setEndDowntrack(prevPoint.getDowntrack());
      currentConflict.setEndTime(prevPoint.getStamp());
      conflicts.add(currentConflict);
    }

    return conflicts;
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath) {
    // Prepare to store conflicts
    List<ConflictSpace> conflicts = new LinkedList<>();
    // Build Map for other path
    NSpatialHashMap otherPathMap = NSpatialHashMapFactory.buildSpatialHashMap(maxSize);
    insertPoints(otherPath, otherPathMap);

    // Iterate over all points in the host path
    ConflictSpace currentConflict = null;
    int lane = 0;
    RoutePointStamped prevPoint = null;

    for (RoutePointStamped routePoint: hostPath) {
      // Get lane
      lane = RouteSegment.fromMessage(routePoint.getSegment()).determinePrimaryLane(routePoint.getDowntrack());

      // Check for collisions
      boolean hasCollision = false;

      if (!otherPathMap.surrounds(routePoint.getPoint())) {
        continue;
      }
      
      // Update conflicts
      if (otherPathMap.getCollisions(routePoint.getPoint()).isEmpty()) { 
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane, routePoint.getSegment());
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          currentConflict.setEndDowntrack(prevPoint.getDowntrack());
          currentConflict.setEndTime(prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane, routePoint.getSegment());
        } 
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          currentConflict.setEndDowntrack(prevPoint.getDowntrack());
          currentConflict.setEndTime(prevPoint.getStamp());
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = routePoint;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      currentConflict.setEndDowntrack(prevPoint.getDowntrack());
      currentConflict.setEndTime(prevPoint.getStamp());
      conflicts.add(currentConflict);
    }

    return conflicts;
  }
}