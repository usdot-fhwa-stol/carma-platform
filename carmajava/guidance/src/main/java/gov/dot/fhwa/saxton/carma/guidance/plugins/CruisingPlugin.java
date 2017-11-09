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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.logging.Log;

/**
 * Cruising Plugin
 * </p>
 * Implements the basic behavior of commanding the speed limit (as specified by RouteManager)
 * at each route segment.
 */
public class CruisingPlugin extends AbstractPlugin {
  protected ISubscriber<RouteState> routeStateSub;
  protected ISubscriber<Route> routeSub;
  protected ISubscriber<RouteSegment> currentSegmentSub;

  protected AtomicReference<Route> currentRoute = new AtomicReference<>();
  protected AtomicReference<RouteSegment> currentSegment = new AtomicReference<>();
  protected List<SpeedLimit> speedLimits = new ArrayList<>();
  protected double maxAccel;
  protected boolean firstTrajectory = true;
  protected static final double MPH_TO_MPS = 0.44704; // From Google calculator
  protected static final double DISTANCE_EPSILON = 0.0001;

  protected class SpeedLimit {
    double speedLimit;
    double location;
  }

  protected class TrajectorySegment {
    double start;
    double end;
    double startSpeed;
    double endSpeed;
  }

  public CruisingPlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Cruising Plugin");
    version.setMajorRevision(0);
    version.setIntermediateRevision(0);
    version.setMinorRevision(1);
  }

  @Override
  public void onInitialize() {
    log.info("Cruisng plugin initializing...");
    routeStateSub = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSub.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
      @Override
      public void onMessage(RouteState msg) {
      }
    });

    routeSub = pubSubService.getSubscriberForTopic("route", Route._TYPE);
    routeSub.registerOnMessageCallback(new OnMessageCallback<Route>() {
      @Override
      public void onMessage(Route msg) {
        currentRoute.set(msg);
        setSpeedLimits(processSpeedLimits(msg));
      }
    });

    currentSegmentSub = pubSubService.getSubscriberForTopic("current_segment", RouteSegment._TYPE);
    currentSegmentSub.registerOnMessageCallback(new OnMessageCallback<RouteSegment>() {
      @Override
      public void onMessage(RouteSegment msg) {
        currentSegment.set(msg);
      }
    });

    maxAccel = pluginServiceLocator.getParameterSource().getDouble("~cruising_max_accel", 3.0);
    log.info("Cruising plugin initialized.");
  }

  @Override
  public void loop() {
    try {
      Thread.sleep(500);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
    }
  }

  @Override
  public void onResume() {
    // NO-OP
  }

  @Override
  public void onSuspend() {
    // NO-OP
  }

  @Override
  public void onTerminate() {
    // NO-OP
  }

  protected double mphToMps(byte milesPerHour) {
    return milesPerHour * MPH_TO_MPS;
  }

  protected boolean fpEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  protected List<SpeedLimit> processSpeedLimits(Route route) {
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

    log.info(String.format("Processed route with %d waypoints over %.2f m", limits.size(), dtdAccum));

    return limits;
  }

  protected void setSpeedLimits(List<SpeedLimit> speedLimits) {
    this.speedLimits = speedLimits;
  }

  // TODO: Improve handling of first and last speed limits on boundaries
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

  protected List<TrajectorySegment> findTrajectoryGaps(Trajectory traj, double trajStartSpeed, double trajEndSpeed) {
    List<IManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
      @Override
      public int compare(IManeuver o1, IManeuver o2) {
        return Double.compare(o1.getStartDistance(), o2.getStartDistance());
      }
    });

    List<TrajectorySegment> gaps = new ArrayList<>();

    // Will be the only case to be handled for now, nothing else should be generating longitudinal maneuvers
    if (longitudinalManeuvers.isEmpty()) {
      TrajectorySegment seg = new TrajectorySegment();
      seg.start = traj.getStartLocation();
      seg.end = traj.getEndLocation();
      seg.startSpeed = trajStartSpeed;
      seg.endSpeed = trajEndSpeed;

      gaps.add(seg);

      return gaps;
    }

    // Find the gaps between
    if (!fpEquals(longitudinalManeuvers.get(0).getStartDistance(), traj.getStartLocation(), DISTANCE_EPSILON)) {
      TrajectorySegment seg = new TrajectorySegment();
      seg.start = traj.getStartLocation();
      seg.end = longitudinalManeuvers.get(0).getStartDistance();
      seg.startSpeed = trajStartSpeed;
      seg.endSpeed = longitudinalManeuvers.get(0).getStartSpeed();

      gaps.add(seg);
    }

    IManeuver prev = null;
    for (IManeuver maneuver : longitudinalManeuvers) {
      if (prev != null) {
        if (!fpEquals(prev.getEndDistance(), maneuver.getStartDistance(), DISTANCE_EPSILON)) {
          TrajectorySegment seg = new TrajectorySegment();
          seg.start = prev.getEndDistance();
          seg.end = maneuver.getStartDistance();
          seg.startSpeed = prev.getTargetSpeed();
          seg.endSpeed = maneuver.getTargetSpeed();
          gaps.add(seg);
        }
      }

      prev = maneuver;
    }

    if (!fpEquals(prev.getEndDistance(), traj.getEndLocation(), DISTANCE_EPSILON)) {
      TrajectorySegment seg = new TrajectorySegment();
      seg.start = prev.getEndDistance();
      seg.end = traj.getEndLocation();
      seg.startSpeed = prev.getTargetSpeed();
      seg.endSpeed = trajEndSpeed;
      gaps.add(seg);
    }

    return gaps;
  }

  protected void planManeuvers(Trajectory t, double startDist, double endDist, double startSpeed, double endSpeed) {
    ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();

    log.info(String.format("Planning maneuver {start=%.02f,end=%.02f,startSpeed=%.02f,endSpeed=%.02f}", startDist,
        endDist, startSpeed, endSpeed));

    double maneuverEnd = startDist;
    if (startSpeed < endSpeed) {
      // Generate a speed-up maneuver
      SpeedUp speedUp = new SpeedUp();
      speedUp.setSpeeds(startSpeed, endSpeed);
      speedUp.setMaxAccel(maxAccel);
      planner.planManeuver(speedUp, startDist);

      // Adjust our accel to complete the maneuver
      if (speedUp.getEndDistance() > endDist) {
        SpeedUp speedUp2 = new SpeedUp();

        speedUp2.setSpeeds(startSpeed, endSpeed);
        speedUp2.setMaxAccel(maxAccel);
        planner.planManeuver(speedUp2, startDist, endDist);
        t.addManeuver(speedUp2);
        maneuverEnd = speedUp2.getEndDistance();
      } else {
        t.addManeuver(speedUp);
        maneuverEnd = speedUp.getEndDistance();
      }

      log.info(String.format("Planned SPEED-UP maneuver from [%.2f, %.2f) m/s over [%.02f, %.2f) m", startSpeed,
          endSpeed, startDist, maneuverEnd));
    } else if (startSpeed > endSpeed) {
      // Generate a slowdown maneuver
      SlowDown slowDown = new SlowDown();
      slowDown.setSpeeds(startSpeed, endSpeed);
      slowDown.setMaxAccel(maxAccel);
      planner.planManeuver(slowDown, startDist);

      // Adjust our accel to complete the maneuver
      if (slowDown.getEndDistance() > endDist) {
        SlowDown slowDown2 = new SlowDown();

        slowDown2.setSpeeds(startSpeed, endSpeed);
        slowDown2.setMaxAccel(maxAccel);
        planner.planManeuver(slowDown2, startDist, endDist);
        t.addManeuver(slowDown2);
        maneuverEnd = slowDown2.getEndDistance();
      } else {
        t.addManeuver(slowDown);
        maneuverEnd = slowDown.getEndDistance();
      }
      log.info(String.format("Planned SLOW-DOWN maneuver from [%.2f, %.2f) m/s over [%.2f, %.2f) m", startSpeed,
          endSpeed, startDist, maneuverEnd));
    }

    // Insert a steady speed maneuver to fill whatever's left
    if (maneuverEnd < endDist) {
      SteadySpeed steady = new SteadySpeed();
      steady.setSpeeds(endSpeed, endSpeed);
      steady.setMaxAccel(maxAccel);
      planner.planManeuver(steady, maneuverEnd);
      steady.overrideEndDistance(endDist);

      t.addManeuver(steady);
      log.info(String.format("Planned STEADY-SPEED maneuver at %.2f m/s over [%.2f, %.2f) m", steady.getTargetSpeed(),
          steady.getStartDistance(), steady.getEndDistance()));
    }
  }

  @Override
  public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {
    List<SpeedLimit> trajLimits = getSpeedLimits(speedLimits, traj.getStartLocation(), traj.getEndLocation());

    // Find the gaps and record the speeds at the boundaries (pass in params for start and end speed)
    List<TrajectorySegment> gaps = null;
    if (trajLimits.size() >= 1) {
      gaps = findTrajectoryGaps(traj, expectedEntrySpeed, trajLimits.get(trajLimits.size() - 1).speedLimit);
    } else {
      gaps = findTrajectoryGaps(traj, expectedEntrySpeed, expectedEntrySpeed);
    }

    if (traj.getLongitudinalManeuvers().size() > 0) {
      if (gaps.size() == 0) {
        log.info("No gaps found to interpolate. Generating no maneuvers.");
        return;
      }

      log.info("Multiple pre-planned maneuvers found, with gaps to fill. Planning interpolating cruising trajectory.");

      for (TrajectorySegment gap : gaps) {
        planManeuvers(traj, gap.start, gap.end, gap.startSpeed, gap.endSpeed);
      }
    } else {
      log.info("No pre-planned longitudinal maneuvers found. Generating trajectory to follow speed limit.");

      if (trajLimits.isEmpty()) {
        // Hold the initial speed for the whole trajectory
        log.info("Planning new trajectory with no speed limits");
        if (firstTrajectory) {
          // Handle the case where we don't yet see a speed limit on our trajectory planning window
          planManeuvers(traj, traj.getStartLocation(), traj.getEndLocation(), expectedEntrySpeed,
              speedLimits.get(0).speedLimit);
          firstTrajectory = false;
        } else {
          planManeuvers(traj, traj.getStartLocation(), traj.getEndLocation(), expectedEntrySpeed, expectedEntrySpeed);
        }
      } else {
        // Take the first speed limit
        SpeedLimit first = trajLimits.get(0);
        log.info("Planning trajectory with initial speed limit: " + first.speedLimit);
        planManeuvers(traj, traj.getStartLocation(), first.location, expectedEntrySpeed, first.speedLimit);

        // Take any intermediary speed limits
        double prevDist = first.location;
        double prevSpeed = first.speedLimit;

        // Usage of DISTANCE_EPSILON here is a bit of a hack
        for (SpeedLimit limit : getSpeedLimits(speedLimits, first.location + DISTANCE_EPSILON, traj.getEndLocation())) {
          log.info("Planning trajectory with speed limit: " + limit.speedLimit);
          planManeuvers(traj, prevDist, limit.location, prevSpeed, limit.speedLimit);
          prevDist = limit.location;
          prevSpeed = limit.speedLimit;
        }

        // Hold the last speed limit until the end of the trajectory
        SpeedLimit last = trajLimits.get(trajLimits.size() - 1);
        log.info("Planning trajectory with last speed limit: " + last.speedLimit);
        planManeuvers(traj, last.location, traj.getEndLocation(), last.speedLimit, last.speedLimit);
      }
    }
  }
}
