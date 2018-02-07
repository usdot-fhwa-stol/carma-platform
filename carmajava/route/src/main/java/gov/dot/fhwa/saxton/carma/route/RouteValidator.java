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

package gov.dot.fhwa.saxton.carma.route;

import org.apache.commons.logging.Log;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Class which represents the abstraction of a travel route which a vehicle will follow.
 * A routes geometry is defined by RouteWaypoints which has lat/lon coordinates.
 */
public class RouteValidator {

  private SaxtonLogger log;
  private static final double MIN_ANGLE_BETWEEN_SEGMENTS_RAD = Math.PI / 2.0;
  private static final double MIN_SEGMENT_LENGTH_M = 1.0;
  private static final double MAX_SEGMENT_LENGTH_M = 1000.0;
  private static final int    MIN_LANE_COUNT = 1;
  private static final double MIN_LANE_WIDTH = 2.0; // Highway lane standard is 3.7m, but might be smaller on side roads
  private static final int    MIN_SPEED_LIMIT_MPH = 0;
  private static final double MIN_ALTITUDE_M = -500.0; // Lowest exposed land on earth is -413 m. Lowest tunnel is around -300 m
  private static final double MAX_ALTITUDE_M = 5500.0; // Highest road in the world is < 5500 m
  private static final double MAX_LATITUDE_ANGLE_DEG = 90.0; 
  private static final double MAX_LONGITUDE_ANGLE_DEG = 180.0; 
  private static final int    MIN_LANE_INDEX = 0;
  private static final double MIN_MILE_MARKER = 0.0;

  /**
   * Constructor which initializes a route from a provided list of waypoints
   *
   * @param waypoints The list of waypoints which will be used to build the route
   * @param routeID   The id to assign to the route. Should be unique
   * @param routeName The display name of the route
   */
  public RouteValidator(Log log) {
    this.log = new SaxtonLogger(this.getClass().getSimpleName(), log);
  }


  /**
   * Determines if a route is valid for use in the CARMA platform
   * The route's validity is marked by setting the route valid flag
   * 
   * @param route The route to validate
   */
  public void validateRoute(Route route) {
    RouteSegment prevSegment = null;
    boolean valid = true;
    log.info("Validating " + route);
    for (RouteSegment seg: route.getSegments()) {
      // Validate segment
      valid = valid && validSegment(seg);

      if (prevSegment != null) {
        // Validate angle between adjacent segments is always less than 90 degrees
        Vector segVec = seg.getLineSegment().getVector();
        Vector prevVec = prevSegment.getLineSegment().getVector();
        double angle = segVec.getAngleBetweenVectors(prevVec);
        if (Math.abs(angle) > MIN_ANGLE_BETWEEN_SEGMENTS_RAD) {
          log.warn("Invalid angle between segments first: " + prevSegment + " second: " + seg);
          valid = false;
        }
      }
      
      prevSegment = seg;
    }

    if (!valid) {
      log.warn("Failed to pass validation for " + route);
    }

    route.setValid(valid);
  }

  /**
   * Returns true if this segment can be considered valid for use in a route
   * @return boolean valid status
   */
  public boolean validSegment(RouteSegment seg) {
    boolean valid = true;
    if (seg.length() < MIN_SEGMENT_LENGTH_M ) {
      log.warn("Invalid segment length below minimum for " + seg);
      valid = false;
    }
    if (seg.length() > MAX_SEGMENT_LENGTH_M) {
      log.warn("Invalid segment length above maximum for " + seg);
      valid = false;
    }
    return valid && validWaypoint(seg.getUptrackWaypoint()) && validWaypoint(seg.getDowntrackWaypoint());
  }

    /**
   * Returns true if this waypoint can be considered valid for use in a route
   * @return boolean valid status
   */
  public boolean validWaypoint(RouteWaypoint wp) {
    // Validate required fields 
    boolean valid = true;
    if (wp.getLaneCount() < MIN_LANE_COUNT) {
      log.warn("Invalid lane count: " + wp.getLaneCount() + " at " + wp); 
      valid = false;
    }
    if (wp.getLaneWidth() < MIN_LANE_WIDTH) {
      log.warn("Invalid lane width: " + wp.getLaneWidth() + " at " + wp); 
      valid = false;
    }
    if (wp.getLowerSpeedLimit() >= wp.getUpperSpeedLimit()) {
      log.warn("Invalid lower speed limit: " + wp.getLowerSpeedLimit() + " upper: " + wp.getUpperSpeedLimit() + " at " + wp); 
      valid = false;
    }
    if (wp.getLowerSpeedLimit() < MIN_SPEED_LIMIT_MPH) {
      log.warn("Invalid lower speed limit: " + wp.getLowerSpeedLimit() + " at " + wp); 
      valid = false;
    }
    if (wp.getMinCrossTrack() >= wp.getMaxCrossTrack()) {
      log.warn("Invalid crosstrack values min: " + wp.getMinCrossTrack() + " max: " + wp.getMaxCrossTrack() + " at " + wp); 
      valid = false;
    }
    if (wp.getMinCrossTrack() > -wp.getLaneWidth() / 2.0) {
      log.warn("Invalid min crosstrack less than half lane width " + wp.getMinCrossTrack() + " lane width: " + wp.getLaneWidth() + " at " + wp); 
      valid = false;
    }
    if (wp.getMaxCrossTrack() < wp.getLaneWidth() / 2.0) {
      log.warn("Invalid max crosstrack less than half lane width" + wp.getMaxCrossTrack() + " lane width: " + wp.getLaneWidth() + " at " + wp); 
      valid = false;
    }
    if (wp.getLocation().getAltitude() < MIN_ALTITUDE_M) {
      log.warn("Invalid altitude below minimum at: " + wp); 
      valid = false;
    }
    if (wp.getLocation().getAltitude() > MAX_ALTITUDE_M) {
      log.warn("Invalid altitude above maximum at: " + wp); 
      valid = false;
    }
    if (Math.abs(wp.getLocation().getLatitude()) > MAX_LATITUDE_ANGLE_DEG) {
      log.warn("Invalid latitude at: " + wp); 
      valid = false;
    }
    if (Math.abs(wp.getLocation().getLongitude()) > MAX_LONGITUDE_ANGLE_DEG) {
      log.warn("Invalid longitude at: " + wp); 
      valid = false;
    }

    // Validate optional fields
    short bitMask = wp.getSetFields();
    if ((bitMask & cav_msgs.RouteWaypoint.LANE_CLOSURES) != 0) {
      boolean validLaneClosures = true;
      for (int lane : wp.getLaneClosures()) {
        validLaneClosures = validLaneClosures && (lane >= MIN_LANE_INDEX);
        if (lane < MIN_LANE_INDEX) {
          log.warn("Invalid lane index of " + lane + " in list of lane closures at " + wp); 
          valid = false;
        }
      }
    }
    if ((bitMask & cav_msgs.RouteWaypoint.NEAREST_MILE_MARKER) != 0) {
      if (wp.getNearestMileMarker() < MIN_MILE_MARKER) {
        log.warn("Invalid nearest mile marker: " + wp.getNearestMileMarker() +" at " + wp); 
        valid = false;
      }
    }
    if ((bitMask & cav_msgs.RouteWaypoint.REQUIRED_LANE_INDEX) != 0) {
      if (wp.getRequiredLaneIndex() < MIN_LANE_INDEX || wp.getRequiredLaneIndex() >= wp.getLaneCount()) {
        log.warn("Invalid required lane index " + wp.getRequiredLaneIndex() + " at " + wp); 
        valid = false;
      }
    }

    return valid;
  }
}
