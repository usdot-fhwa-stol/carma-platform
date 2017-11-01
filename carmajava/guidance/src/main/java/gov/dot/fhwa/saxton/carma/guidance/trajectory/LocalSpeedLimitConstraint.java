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
import java.util.ArrayList;

/**
 * LocalSpeedLimitConstraint ensures that the planned Trajectory never exceeds the speed limit configured in the
 * selected Route.
 */
public class LocalSpeedLimitConstraint implements TrajectoryValidationConstraint {
  protected List<SpeedLimit> speedLimits;
  protected List<IManeuver> offendingManeuvers;

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
    }

    return limits;
  }
  // END LOGIC BORROWED FROM CRUISING PLUGIN

  protected SpeedLimit getLimitAtDistance(double dist) {
    SpeedLimit last = null;
    for (SpeedLimit limit : speedLimits) {
      if (limit.location > dist) {
        return last;
      }

      last = limit;
    }

    return last;
  }

	@Override
	public void visit(IManeuver maneuver) {
    if (!(maneuver instanceof LongitudinalManeuver)) {
      return;
    }

    boolean startSpeedLegal = maneuver.getStartSpeed() <= getLimitAtDistance(maneuver.getStartDistance()).speedLimit;
    boolean endSpeedLegal = maneuver.getTargetSpeed() <= getLimitAtDistance(maneuver.getEndDistance()).speedLimit;

    if (!(startSpeedLegal && endSpeedLegal)) {
      offendingManeuvers.add(maneuver);
    }
	}

	@Override
	public TrajectoryValidationResult getResult() {
    if (offendingManeuvers.isEmpty()) {
      reset();
      return new TrajectoryValidationResult();
    } else {
      TrajectoryValidationResult out = new TrajectoryValidationResult(new TrajectoryValidationError("Maneuvers exceed route defined Speed Limit", offendingManeuvers));
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


