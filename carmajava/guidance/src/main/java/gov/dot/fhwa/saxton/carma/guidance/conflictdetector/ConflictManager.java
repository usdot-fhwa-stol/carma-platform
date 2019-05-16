/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.ISpatialStructure;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.ISpatialStructureFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.route.Route;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

/**
 * Class Maintains a tracked set of external vehicle paths from MobilityPath and
 * MobilityRequests messages The set of paths can be queried for collisions.
 * 
 * Collision detection is done with an injected {@link ISpatialStructure} The path sets are
 * synchronized making this class Thread-Safe
 * 
 * The times stamps used on paths should all be referenced to the same origin
 * The current time information is provided by a passed in
 * {@link IMobilityTimeProvider}
 */
public class ConflictManager implements IConflictManager {
  private final ISpatialStructureFactory structureFactory;
  // Dimensions used for collision detection around point
  private final double downtrackMargin;
  private final double crosstrackMargin;
  private final double timeMargin;
  private final double lateralBias;
  private final double longitudinalBias;
  private final double temporalBias;
  // The tracked paths
  private final Map<String, ISpatialStructure> mobilityPathSpatialMaps = Collections.synchronizedMap(new HashMap<>());
  private final Map<String, ISpatialStructure> requestedPathSpatialMaps = Collections.synchronizedMap(new HashMap<>());
  private final Map<String, String> planIdMap = Collections.synchronizedMap(new HashMap<>());
  // Time provider
  private final IMobilityTimeProvider timeProvider;
  // Route
  private Route route;
  ILogger log;

  /**
   * Constructor
   * 
   * @param structureFactory Factory used to produce spatial structures used for collision checking
   * @param downtrackMargin  The downtrack distance margin within which a point
   *                         will be considered in collision
   * @param crosstrackMargin The crosstrack distance margin within which a point
   *                         will be considered in collision
   * @param timeMargin       The time margin in seconds within which a point will
   *                         be considered in collision
   * @param lateralBias      The percentage of the crosstrack margin to bias the
   *                         host vehicle's bounding box to the right
   * @param longitudinalBias The percentage of the downtrack margin to bias the
   *                         host vehicle's bounding box to the front
   * @param temporalBias     The percentage of the time margin to bias the host
   *                         vehicle's bounding box to the future
   * @param timeProvider     The object responsible to determining the time used
   *                         in mobility messages
   */
  public ConflictManager(ISpatialStructureFactory structureFactory, double downtrackMargin, double crosstrackMargin, double timeMargin,
      double lateralBias, double longitudinalBias, double temporalBias, IMobilityTimeProvider timeProvider) {

    this.structureFactory = structureFactory;
    this.downtrackMargin = downtrackMargin;
    this.crosstrackMargin = crosstrackMargin;
    this.timeMargin = timeMargin;
    this.timeProvider = timeProvider;
    this.lateralBias = lateralBias;
    this.longitudinalBias = longitudinalBias;
    this.temporalBias = temporalBias;
    this.log = LoggerManager.getLogger();
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

  @Override
  public boolean addMobilityPath(List<RoutePointStamped> path, String vehicleStaticId) {
    log.info("Adding mobility path");
    if (path == null || path.isEmpty() || vehicleStaticId == null) {
      return false;
    }
    synchronized (mobilityPathSpatialMaps) {
      addPath(path, vehicleStaticId, mobilityPathSpatialMaps);
    }
    return true;
  }

  @Override
  public boolean addRequestedPath(List<RoutePointStamped> path, String planId, String vehicleId) {
    log.info("Adding requested path");
    if (path == null || path.isEmpty() || planId == null || vehicleId == null) {
      return false;
    }
    synchronized (requestedPathSpatialMaps) {
      addPath(path, planId, requestedPathSpatialMaps);
      planIdMap.put(planId, vehicleId);
    }
    return true;
  }

  /**
   * Helper function for adding paths to a map of spatial hash maps
   * 
   * @param path The path to add for future conflict detection
   * @param key  The key to use for identifying this path
   * 
   * @return True if the path could be added. False if not.
   */
  private void addPath(List<RoutePointStamped> path, String key, Map<String, ISpatialStructure> map) {
    log.info("addPath");
    // Get current path for this vehicle
    ISpatialStructure vehiclesPath = map.get(key);
    // If not current path for this vehicle add it
    if (vehiclesPath == null) {
      log.info("Creating new spatial hash map");
      vehiclesPath = structureFactory.buildSpatialStructure();
      map.put(key, vehiclesPath);
    }
    log.info("Preparing to insert points");
    // Insert points
    long time0 = System.currentTimeMillis();
    insertPoints(path, vehiclesPath, downtrackMargin, crosstrackMargin, timeMargin);
    long time1 = System.currentTimeMillis();
    log.debug("addPath: call to insertPoints took " + (time1 - time0) + " ms.");
  }

  /**
   * Helper function inserts a list of RoutePointStamped into a ISpatialStructure
   * 
   * @param path The points to insert
   * @param map  The map to insert points into
   * @param downtrackMargin The downtrack margin to use around points
   * @param crosstrackMargin The crosstrack margin to use around points
   * @param timeMargin The time margin to use around points
   */
  private void insertPoints(List<RoutePointStamped> path, ISpatialStructure map, double downtrackMargin, double crosstrackMargin, double timeMargin) {
    // Add points to spatial map
    log.debug("Inserting path with size " + path.size());
    int count = 0;
    for (RoutePointStamped routePoint : path) {
      // log.info("Inserting point " + count + ": " + routePoint.getPoint());
      // Define bounds
      Point3D minBoundingPoint = new Point3D(routePoint.getDowntrack() - downtrackMargin,
          routePoint.getCrosstrack() - crosstrackMargin, routePoint.getStamp() - timeMargin);
      Point3D maxBoundingPoint = new Point3D(routePoint.getDowntrack() + downtrackMargin,
          routePoint.getCrosstrack() + crosstrackMargin, routePoint.getStamp() + timeMargin);
      // Insert point
      map.insert(new CartesianObject(Arrays.asList(minBoundingPoint, maxBoundingPoint)));
      // log.info("Inserted point");
      count++;
    }
    log.debug("Done inserting");
  }

  @Override
  public boolean removeMobilityPath(String vehicleStaticId) {
    synchronized (mobilityPathSpatialMaps) {
      return mobilityPathSpatialMaps.remove(vehicleStaticId) != null;
    }
  }

  @Override
  public boolean removeRequestedPath(String planId) {
    synchronized (requestedPathSpatialMaps) {
      planIdMap.remove(planId);
      return requestedPathSpatialMaps.remove(planId) != null;
    }
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath) {
    log.info("Getting any conflicts with host path");
    if (hostPath == null || hostPath.isEmpty() || route == null) {
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

    for (RoutePointStamped routePoint : hostPath) {
      // If the provided point occurs before the current time. There is no point in
      // evaluating it
      if (routePoint.getStamp() < minTime) {
        continue;
      }
      // Get lane
      lane = route.getSegments().get(routePoint.getSegmentIdx()).determinePrimaryLane(routePoint.getCrosstrack());

      // Check for collisions with mobility paths
      List<String> conflictingVehicles = hasCollision(mobilityPathSpatialMaps, routePoint, minTime);
      if (conflictingVehicles.isEmpty()) {
        // Check for collisions with requested paths
        conflictingVehicles = hasCollision(requestedPathSpatialMaps, routePoint, minTime);
      }
      // Update conflicts
      if (!conflictingVehicles.isEmpty()) {
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane,
              routePoint.getSegmentIdx());
          currentConflict.addConflictingVehicles(conflictingVehicles);
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict
          // and create a new one
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to
          // define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane,
              routePoint.getSegmentIdx());
          currentConflict.addConflictingVehicles(conflictingVehicles);
        }
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict
        // is done
        if (currentConflict != null) {
          currentConflict.addConflictingVehicles(conflictingVehicles);
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
   * Helper function returns true if any of the spatial maps in the provided map
   * contain elements which collide with the provided point
   * 
   * @param mapContainer The set of spatial hash maps which will be evaluated
   * @param routePoint   The point to check for collisions
   * @param minTime      The minimum time in seconds which is still valid for
   *                     consideration
   * 
   * @return A list static ids for vehicles which the provided point conflict
   *         with. The list is empty if no conflict exist
   */
  private List<String> hasCollision(Map<String, ISpatialStructure> mapContainer, RoutePointStamped routePoint,
      double minTime) {
    final int TIME_IDX = 2, MAX_IDX = 1;
    List<String> conflictingVehicles = new LinkedList<>();
    // Directly access iterator for safe removal while iterating
    synchronized (mapContainer) {
      for (Iterator<Entry<String, ISpatialStructure>> i = mapContainer.entrySet().iterator(); i.hasNext();) {
        Entry<String, ISpatialStructure> entry = i.next();
        String id = entry.getKey();
        ISpatialStructure map = entry.getValue();

        // If the maximum time of the current map is before the minimum time being
        // evaluated
        // it should be skipped and removed from future consideration
        if (minTime > map.getBounds()[TIME_IDX][MAX_IDX]) {
          i.remove();
          // If this is the a requested path disassociate the plan and vehicle ids
          if (mapContainer == requestedPathSpatialMaps) {
            planIdMap.remove(id);
          }
          continue;
        }
        // If this map contains the point being evaluated collisions must be checked for
        RoutePointStamped transformed = new RoutePointStamped(
            routePoint.getDowntrack() + (downtrackMargin * longitudinalBias),
            routePoint.getCrosstrack() + (crosstrackMargin * lateralBias),
            routePoint.getStamp() + (timeMargin * temporalBias));

        Point3D minBoundingPoint = new Point3D(transformed.getDowntrack() - downtrackMargin,
            transformed.getCrosstrack() - crosstrackMargin, transformed.getStamp() - timeMargin);
        Point3D maxBoundingPoint = new Point3D(transformed.getDowntrack() + downtrackMargin,
            transformed.getCrosstrack() + crosstrackMargin, transformed.getStamp() + timeMargin);

        List<Point3D> pointCloud = new ArrayList<>();
        pointCloud.add(minBoundingPoint);
        pointCloud.add(maxBoundingPoint);
        CartesianObject boundingBox = new CartesianObject(pointCloud);
        if (map.surrounds(transformed.getPoint())) {
          if (!map.getCollisions(boundingBox).isEmpty()) {
            // Get the vehicle id and add it to list of conflicting ids
            if (mapContainer == requestedPathSpatialMaps) {
              conflictingVehicles.add(planIdMap.get(id));
            } else {
              conflictingVehicles.add(id);
            }
            continue;
          }
        }
      }
    }
    return conflictingVehicles;
  }

  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath) {
    return getConflicts(hostPath, otherPath, structureFactory.buildSpatialStructure(), downtrackMargin, crosstrackMargin, timeMargin, longitudinalBias, lateralBias, temporalBias);
  }

  // The following variables are for debugging only
  // public static long nanoSecBuilding = 0;
  // public static long nanoSecChecking = 0;

  // Note the parameters in this function are overriding the class members by the same name
  @Override
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath, ISpatialStructure spatialStructure,
    double downtrackMargin, double crosstrackMargin, double timeMargin, double longitudinalBias, double lateralBias, double temporalBias) {
    log.debug("Getting conflicts between two paths");
    if (hostPath == null || otherPath == null || hostPath.isEmpty() || otherPath.isEmpty()) {
      return new LinkedList<>();
    }

    //System.out.println("Host: " + hostPath.size());
    //System.out.println("Other: " + otherPath.size());

    long startTime = System.nanoTime();
    // Prepare to store conflicts
    List<ConflictSpace> conflicts = new LinkedList<>();
    // Build Map for other path
    insertPoints(otherPath, spatialStructure, downtrackMargin, crosstrackMargin, timeMargin);
    
    // nanoSecBuilding += (System.nanoTime()- startTime);
    startTime = System.nanoTime();

    // Iterate over all points in the host path
    ConflictSpace currentConflict = null;
    int lane = 0;
    RoutePointStamped prevPoint = null;

    for (RoutePointStamped routePoint : hostPath) {
      RoutePointStamped transformed = new RoutePointStamped(
          routePoint.getDowntrack() + (downtrackMargin * longitudinalBias),
          routePoint.getCrosstrack() + (crosstrackMargin * lateralBias),
          routePoint.getStamp() + (timeMargin * temporalBias));

      Point3D minBoundingPoint = new Point3D(transformed.getDowntrack() - downtrackMargin,
          transformed.getCrosstrack() - crosstrackMargin, transformed.getStamp() - timeMargin);
      Point3D maxBoundingPoint = new Point3D(transformed.getDowntrack() + downtrackMargin,
          transformed.getCrosstrack() + crosstrackMargin, transformed.getStamp() + timeMargin);

      // Get lane
      lane = route.getSegments().get(transformed.getSegmentIdx()).determinePrimaryLane(transformed.getCrosstrack());

      List<Point3D> pointCloud = new ArrayList<>();
      pointCloud.add(minBoundingPoint);
      pointCloud.add(maxBoundingPoint);
      CartesianObject boundingBox = new CartesianObject(pointCloud);

      // Update conflicts
      if (!spatialStructure.getCollisions(boundingBox).isEmpty()) {
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(routePoint.getDowntrack(), routePoint.getStamp(), lane,
              routePoint.getSegmentIdx());
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict
          // and create a new one
          closeConflict(currentConflict, prevPoint.getDowntrack(), prevPoint.getStamp());
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to
          // define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDowntrack(), prevPoint.getStamp(), lane,
              routePoint.getSegmentIdx());
        }
      } else {
        // If there were no conflicts but we are tracking a conflict then that conflict
        // is done
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

    // nanoSecChecking += (System.nanoTime() - startTime);

    return conflicts;
  }

  /**
   * Helper function to close a conflict while ensuring it has some non-zero width
   * 
   * @param conflict     The conflict to close. Modified in place to have minimum
   *                     dimensions equal to the collision margins
   * @param endDowntrack The ending downtrack distance
   * @param endTime      the ending time
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
