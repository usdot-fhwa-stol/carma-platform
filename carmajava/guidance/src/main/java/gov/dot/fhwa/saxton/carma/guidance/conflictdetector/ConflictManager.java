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
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.function.BiConsumer;

import org.jboss.netty.util.internal.ConcurrentHashMap;

import cav_msgs.MobilityPath;
import cav_msgs.Route;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

/**
 * Class Maintains a tracked set of external vehicle paths from MobilityPath and MobilityRequests messages
 * The set of paths can be queried for collisions.
 * 
 * Collision detection is done with a {@link NSpatialHashMap}
 * The path sets are synchronized making this class Thread-Safe
 * 
 * The times stamps used on paths should all be referenced to the same origin
 * The current time information is provided by a passed in {@link IMobilityTimeProvider}
 */
public class ConflictManager implements IConflictManager {
  // The maximum size of a cell in the map
  private final double[] cellSize;
  // Dimensions used for collision detection around point
  private final double downtrackMargin;
  private final double crosstrackMargin;
  private final double timeMargin;
  // The tracked paths
  private final Map<String, NSpatialHashMap> mobilityPathSpatialMaps = Collections.synchronizedMap(new HashMap<>());
  private final Map<String, NSpatialHashMap> requestedPathSpatialMaps = Collections.synchronizedMap(new HashMap<>());
  // Time provider
  private final IMobilityTimeProvider timeProvider;

  /**
   * Constructor
   * 
   * @param cellSize 3 element array of cell sizes for collision lookup
   * @param downtrackMargin The downtrack distance margin within which a point will be considered in collision
   * @param crosstrackMargin The crosstrack distance margin within which a point will be considered in collision
   * @param timeMargin The time margin in seconds within which a point will be considered in collision
   * @param timeProvider The object responsible to determining the time used in mobility messages
   */
  public ConflictManager(double[] cellSize, double downtrackMargin, 
   double crosstrackMargin, double timeMargin, IMobilityTimeProvider timeProvider) {

    this.cellSize = cellSize;
    this.downtrackMargin = downtrackMargin;
    this.crosstrackMargin = crosstrackMargin;
    this.timeMargin = timeMargin;
    this.timeProvider = timeProvider;
  }

  @Override
  public boolean addMobilityPath(List<RoutePointStamped> path, String vehicleStaticId) {
    if (path == null || path.isEmpty() || vehicleStaticId == null) {
      return false;
    }
    addPath(path, vehicleStaticId, mobilityPathSpatialMaps);
    return true;
  }

  @Override
  public boolean addRequestedPath(List<RoutePointStamped> path, String planId) {
    if (path == null || path.isEmpty() || planId == null) {
      return false;
    }
    addPath(path, planId, requestedPathSpatialMaps);
    return true;
  }
  
  /**
   * Helper function fro adding paths to a map of spatial hash maps
   * 
   * @param path The path to add for future conflict detection
   * @param key The key to use for identifying this path
   * 
   * @return True if the path could be added. False if not.
   */
  private void addPath(List<RoutePointStamped> path, String key, Map<String, NSpatialHashMap> map) {
    // Get current path for this vehicle
    NSpatialHashMap vehiclesPath = map.get(key);
    // If not current path for this vehicle add it 
    if (vehiclesPath == null) {
      vehiclesPath =  NSpatialHashMapFactory.buildSpatialHashMap(cellSize);
      map.put(key, vehiclesPath);
    }
    // Insert points
    insertPoints(path, vehiclesPath);
  }

  /**
   * Helper function inserts a list of RoutePointStamped into a NSpatialHashMap
   * 
   * @param path The points to insert
   * @param map The map to insert points into
   */
  private void insertPoints(List<RoutePointStamped> path, NSpatialHashMap map) {
    // Add points to spatial map
    for (RoutePointStamped routePoint: path) {
      // Define bounds
      Point3D minBoundingPoint = new Point3D(
        routePoint.getDowntrack() - downtrackMargin,
        routePoint.getCrosstrack() - crosstrackMargin,
        routePoint.getStamp() - timeMargin);
      Point3D maxBoundingPoint = new Point3D(
        routePoint.getDowntrack() + downtrackMargin,
        routePoint.getCrosstrack() + crosstrackMargin,
        routePoint.getStamp() + timeMargin);
      // Insert point
      map.insert(new CartesianObject(Arrays.asList(minBoundingPoint, maxBoundingPoint)));
    }
  }
  
  @Override
  public boolean removeMobilityPath(String vehicleStaticId) {
    return mobilityPathSpatialMaps.remove(vehicleStaticId) != null;
  }

  @Override
  public boolean removeRequestedPath(String planId) {
    return requestedPathSpatialMaps.remove(planId) != null;
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath) {
    if (hostPath == null || hostPath.isEmpty()) {
      return new LinkedList<>();
    }
    // Prepare to store conflicts
    List<ConflictSpace> conflicts = new LinkedList<>();

    // Iterate over all points in the host path
    ConflictSpace currentConflict = null;
    int lane = 0;
    RoutePointStamped prevPoint = null;
    // Get the minimum time stamp which is still viable
    double minTime = timeProvider.getCurrentTimeSeconds();
    
    for (RoutePointStamped routePoint: hostPath) {
      // If the provided point occurs before the current time. There is no point in evaluating it
      if (routePoint.getStamp() < minTime) {
        continue;
      }
      // Get lane
      lane = RouteSegment.fromMessage(routePoint.getSegment()).determinePrimaryLane(routePoint.getCrosstrack());

      // Check for collisions with mobility paths
      boolean foundCollision = hasCollision(mobilityPathSpatialMaps, routePoint, minTime);
      if (!foundCollision) {
        // Check for collisions with requested paths
        foundCollision = hasCollision(requestedPathSpatialMaps, routePoint, minTime);
      }
      // Update conflicts
      if (foundCollision) { 
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane, routePoint.getSegment());
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane, routePoint.getSegment());
        } 
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = routePoint;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
      conflicts.add(currentConflict);
    }

    return conflicts;
  }

  /**
   * Helper function returns true 
   * if any of the spatial maps in the provided map contain elements which collide with the provided point
   * 
   * @param mapContainer The set of spatial hash maps which will be evaluated
   * @param routePoint The point to check for collisions
   * @param minTime The minimum time in seconds which is still valid for consideration
   * 
   * @return True if a collision was found. False if no collision was found
   */
  private boolean hasCollision(Map<String, NSpatialHashMap> mapContainer, RoutePointStamped routePoint, double minTime) {
    final int TIME_IDX = 2, MAX_IDX = 1;
    boolean hasCollision = false;

    // Directly access iterator for safe removal while iterating
    for(Iterator<Entry<String, NSpatialHashMap>> i = mapContainer.entrySet().iterator(); i.hasNext();) {
      Entry<String, NSpatialHashMap> entry = i.next();
      String vehicleId = entry.getKey();
      NSpatialHashMap map = entry.getValue();

      // If the maximum time of the current map is before the minimum time being evaluated
      // it should be skipped and removed from future consideration
      if (minTime > map.getBounds()[TIME_IDX][MAX_IDX]) {
        i.remove();
        continue;
      }
      // If this map contains the point being evaluated collisions must be checked for
      if (map.surrounds(routePoint.getPoint())) {
        if (!map.getCollisions(routePoint.getPoint()).isEmpty()) {
          hasCollision = true;
          break;
        }
      }
    }
    return hasCollision;
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath) {
    if (hostPath == null || otherPath == null || hostPath.isEmpty() || otherPath.isEmpty()) {
      return new LinkedList<>();
    }
    // Prepare to store conflicts
    List<ConflictSpace> conflicts = new LinkedList<>();
    // Build Map for other path
    NSpatialHashMap otherPathMap = NSpatialHashMapFactory.buildSpatialHashMap(cellSize);
    insertPoints(otherPath, otherPathMap);

    // Iterate over all points in the host path
    ConflictSpace currentConflict = null;
    int lane = 0;
    RoutePointStamped prevPoint = null;

    for (RoutePointStamped routePoint: hostPath) {
      // Get lane
      lane = RouteSegment.fromMessage(routePoint.getSegment()).determinePrimaryLane(routePoint.getCrosstrack());
      
      // Update conflicts
      if (!otherPathMap.getCollisions(routePoint.getPoint()).isEmpty()) { 
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane, routePoint.getSegment());
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane, routePoint.getSegment());
        } 
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = routePoint;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
      conflicts.add(currentConflict);
    }

    return conflicts;
  }

  /**
   * Helper function to close a conflict while ensuring it has some non-zero width
   * 
   * @param conflict The conflict to close. Modified in place to have minimum dimensions equal to the collision margins
   * @param endDowntrack The ending downtrack distance
   * @param endTime the ending time
   */
  void closeConflict(ConflictSpace conflict, double endDowntrack, double endTime) {
    if (endDowntrack - conflict.getStartDowntrack() < downtrackMargin) {
      endDowntrack = conflict.getStartDowntrack() + downtrackMargin;
    }
    conflict.setEndDowntrack(endDowntrack);

    if (endTime - conflict.getStartTime() < timeMargin) {
      endTime = conflict.getStartTime() + timeMargin;
    }
    conflict.setEndTime(endTime);
  }
}
