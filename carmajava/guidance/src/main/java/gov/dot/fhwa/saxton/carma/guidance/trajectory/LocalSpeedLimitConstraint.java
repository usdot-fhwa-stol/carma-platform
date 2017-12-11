/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import java.util.List;
import cav_msgs.Route;
import cav_msgs.RouteSegment;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.ArrayList;

/**
 * LocalSpeedLimitConstraint ensures that the planned Trajectory never exceeds the speed limit configured in the
 * selected Route.
 */
public class LocalSpeedLimitConstraint implements TrajectoryValidationConstraint {
  protected List<SpeedLimit> speedLimits;
  protected List<IManeuver> offendingManeuvers;
  private static final double DISTANCE_EPSILON = 0.0001;
  protected ILogger log = LoggerManager.getLogger();

  public LocalSpeedLimitConstraint(Route route) {
    speedLimits = processRoute(route);
    offendingManeuvers = new ArrayList<>();
  }

  /*
   * BEGIN LOGIC BORROWED FROM CRUISING PLUGIN
   * IF NEEDED IN ADDITIONAL LOCATIONS CONSIDER REFACTORING INTO UTILS CLASS
   */
  protected static final double MPH_TO_MPS = 0.44704; // From Google calculator

  protected class SpeedLimit {
    double speedLimit;
    double location;

    public String toString() {
      return String.format("SpeedLimit={location=%.02f, limit=%.02f}", location, speedLimit);
    }
  }

  protected double mphToMps(byte milesPerHour) {
    return milesPerHour * MPH_TO_MPS;
  }

  private List<SpeedLimit> processRoute(Route route) {
    List<SpeedLimit> limits = new ArrayList<>();

    // Walk the segment list getting the start points speed limits
    double dtdAccum = 0.0;
    for (RouteSegment seg : route.getSegments()) {
      dtdAccum += seg.getLength();
      SpeedLimit limit = new SpeedLimit();
      limit.location = dtdAccum;
      limit.speedLimit = mphToMps(seg.getWaypoint().getSpeedLimit());
      limits.add(limit);
      log.info("Loaded: " + limit);
    }

    return limits;
  }

  protected List<SpeedLimit> getSpeedLimits(List<SpeedLimit> speedLimits, double startDistance, double endDistance) {
    // Get all the speed limits spanned by [startDistance, endDistance)
    List<SpeedLimit> spanned = new ArrayList<>();
    for (SpeedLimit limit : speedLimits) {
      if (limit.location >= startDistance && limit.location < endDistance) {
        spanned.add(limit);
      }
    }

    return spanned;
  }
  // END LOGIC BORROWED FROM CRUISING PLUGIN

  protected SpeedLimit getLimitAtDistance(double dist) {
    for (SpeedLimit limit : speedLimits) {
      if (limit.location > dist) {
        return limit;
      }
    }

    return null;
  }

  @Override
  public void visit(IManeuver maneuver) {
    if (!(maneuver instanceof LongitudinalManeuver)) {
      return;
    }

    // Special case for slowdowns, we only care about where they end up
    LongitudinalManeuver lonMvr = (LongitudinalManeuver) maneuver;
    if (lonMvr instanceof SlowDown) {
      SpeedLimit end = getLimitAtDistance(lonMvr.getEndDistance() - DISTANCE_EPSILON);

      if (end == null) {
        log.warn(String.format("Maneuver from [%.02f, %.02f) deemed illegal due to missing speed limits",
            lonMvr.getStartDistance(), lonMvr.getEndDistance()));
        offendingManeuvers.add(lonMvr);
        return;
      }
     
      if (lonMvr.getTargetSpeed() >= lonMvr.getStartSpeed()) {
        log.warn(String.format("Slowdown lonMvr from [%.02f, %.02f) deemed illegal. end speed >= start_speed",
            lonMvr.getStartDistance(), lonMvr.getEndDistance()));
        offendingManeuvers.add(lonMvr);
      }

      return;
    }

    SpeedLimit start = getLimitAtDistance(lonMvr.getStartDistance());
    SpeedLimit end = getLimitAtDistance(lonMvr.getEndDistance() - DISTANCE_EPSILON);

    if (start == null || end == null) {
      log.warn(String.format("Maneuver from [%.02f, %.02f) deemed illegal due to missing speed limits",
          lonMvr.getStartDistance(), lonMvr.getEndDistance()));
      offendingManeuvers.add(lonMvr);
      return;
    }

    boolean startSpeedLegal = lonMvr.getStartSpeed() <= start.speedLimit;
    boolean endSpeedLegal = lonMvr.getTargetSpeed() <= end.speedLimit;

    if (!(startSpeedLegal && endSpeedLegal)) {
      log.warn(String.format("Maneuver from [%.02f, %.02f) deemed illegal. Start limit = %.02f, End limit = %.02f",
          lonMvr.getStartDistance(), lonMvr.getEndDistance(), start.speedLimit, end.speedLimit));
      offendingManeuvers.add(lonMvr);
    }

    // Now check to see if any spanned limits are illegal, may sometimes be redundant with above
    List<SpeedLimit> spanned = getSpeedLimits(speedLimits, lonMvr.getStartDistance(),
        lonMvr.getEndDistance() - DISTANCE_EPSILON);
    boolean spannedLegal = true;
    for (SpeedLimit limit : spanned) {
      double distFactor = (limit.location - lonMvr.getStartDistance())
          / (lonMvr.getEndDistance() - lonMvr.getStartDistance());
      double deltaV = lonMvr.getTargetSpeed() - lonMvr.getStartSpeed();
      double interpolatedSpeed = lonMvr.getStartSpeed() + (deltaV * distFactor);
      spannedLegal = spannedLegal && interpolatedSpeed <= limit.speedLimit;
      if (interpolatedSpeed > limit.speedLimit) {
        log.warn(String.format(
            "Maneuver from [%.02f, %.02f) deemed illegal using interpolated speed. Distance = %.02f, Interpolated speed = %.02f, Limit = %.02f",
            lonMvr.getStartDistance(), lonMvr.getEndDistance(), limit.location, interpolatedSpeed,
            limit.speedLimit));
      }
    }
  }

  @Override
  public TrajectoryValidationResult getResult() {
    if (offendingManeuvers.isEmpty()) {
      reset();
      return new TrajectoryValidationResult();
    } else {
      TrajectoryValidationResult out = new TrajectoryValidationResult(
          new TrajectoryValidationError("Maneuvers exceed route defined Speed Limit", offendingManeuvers));
      reset();
      return out;
    }
  }

  /**
   * Reset the state so the constraint can accept another Trajectory
   */
  private void reset() {
    offendingManeuvers = new ArrayList<>();
  }
}
