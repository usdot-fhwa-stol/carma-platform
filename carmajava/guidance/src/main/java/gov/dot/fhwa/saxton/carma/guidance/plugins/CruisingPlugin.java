/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.logging.Log;

/**
 * Cruising Plugin
 * </p>
 * Implements the basic behavior of commmanding the speed limit (as specified by RouteManager)
 * at each route segment.
 */
public class CruisingPlugin extends AbstractPlugin {
  protected String name = "Cruising";
  protected String versionId = "v00.00.01";
  protected ISubscriber<RouteState> routeStateSub;
  protected ISubscriber<Route> routeSub;
  protected ISubscriber<RouteSegment> currentSegmentSub;

  protected AtomicReference<Route> currentRoute;
  protected AtomicReference<RouteSegment> currentSegment;
  protected List<SpeedLimit> speedLimits;
  protected double maxAccel;
  protected static final double MPH_TO_MPS = 0.44704; // From Google calculator

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
        speedLimits = processSpeedLimits(msg);
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

  protected List<SpeedLimit> processSpeedLimits(Route route) {
    List<SpeedLimit> limits = new ArrayList<>();

    // Walk the segment list getting the start points speed limits
    double dtdAccum = 0.0;
    for (RouteSegment seg : route.getSegments()) {
      SpeedLimit limit = new SpeedLimit();
      limit.location = dtdAccum;
      limit.speedLimit = mphToMps(seg.getPrevWaypoint().getSpeedLimit());
      limits.add(limit);
      dtdAccum += seg.getLength();
    }

    // If we processed at least one waypoint, get the last waypoint, the endpoint of the last segment
    if (!limits.isEmpty()) {
      List<RouteSegment> segments = route.getSegments();
      RouteSegment lastSeg = segments.get(segments.size());
      SpeedLimit limit = new SpeedLimit();
      limit.location = dtdAccum;
      limit.speedLimit = mphToMps(lastSeg.getWaypoint().getSpeedLimit());
    }

    return limits;
  }

  // TODO: Improve handling of first and last speed limits on boundaries
  protected List<SpeedLimit> getSpeedLimits(double startDistance, double endDistance) {
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
    }

    // Find the gaps between
    if (longitudinalManeuvers.get(0).getStartDistance() != traj.getStartLocation()) {
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
        if (prev.getEndDistance() != maneuver.getStartDistance()) {
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

    if (prev.getEndDistance() != traj.getEndLocation()) {
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

    double maneuverEnd = startDist;
    if (startSpeed < endSpeed) {
      // Generate a speed-up maneuver
      SpeedUp speedUp = new SpeedUp();
      speedUp.setSpeeds(startSpeed, endSpeed);
      speedUp.setMaxAccel(maxAccel);
      planner.planManeuver(speedUp, startDist);
      t.addManeuver(speedUp);

      maneuverEnd = speedUp.getEndDistance();
    } else if (startSpeed > endSpeed) {
      // Generate a slowdown maneuver
      SlowDown slowDown = new SlowDown();
      slowDown.setSpeeds(startSpeed, endSpeed);
      slowDown.setMaxAccel(maxAccel);
      planner.planManeuver(slowDown, startDist);

      maneuverEnd = slowDown.getEndDistance();
      t.addManeuver(slowDown);
    }

    // Insert a steady speed maneuver to fill whatever's left
    if (maneuverEnd < endDist) {
      SteadySpeed steady = new SteadySpeed();
      steady.setSpeeds(endSpeed, endSpeed);
      steady.setMaxAccel(maxAccel);
      planner.planManeuver(steady, maneuverEnd);
      steady.overrideEndDistance(endDist);

      t.addManeuver(steady);
    }
  }
  
  @Override
  public void planTrajectory(Trajectory traj, double trajStartSpeed) {
    List<SpeedLimit> trajLimits = getSpeedLimits(traj.getStartLocation(), traj.getEndLocation());

    // Find the gaps and record the speeds at the boundaries (pass in params for start and end speed)
    List<TrajectorySegment> gaps = null;
    if (trajLimits.size() >= 1) {
       gaps = findTrajectoryGaps(traj, trajStartSpeed, trajLimits.get(trajLimits.size() - 1).speedLimit);
    } else {
      gaps = findTrajectoryGaps(traj, trajStartSpeed, trajStartSpeed);
    }

    if (traj.getLongitudinalManeuvers().size() > 0) {
      log.info("Multiple pre-planned maneuvers found, with gaps to fill. Planning interpolating cruising trajectory.");

      if (gaps.size() == 0) {
        log.info("No gaps found to interpolate. Generating no maneuvers.");
        return;
      }

      for (TrajectorySegment gap : gaps) {
        planManeuvers(traj, gap.start, gap.end, gap.startSpeed, gap.endSpeed);
      }
    } else {
      log.info("No pre-planned longitudinal maneuvers found. Generating trajectory to follow speed limit.");

      if (trajLimits.isEmpty()) {
        // Hold the initial speed for the whole trajectory
        planManeuvers(traj, traj.getStartLocation(), traj.getEndLocation(), trajStartSpeed, trajStartSpeed);
      } else {
        // Take the first speed limit
        SpeedLimit first = trajLimits.get(0);
        planManeuvers(traj, traj.getStartLocation(), first.location, trajStartSpeed, first.speedLimit);

        // Take any intermediary speed limits
        double prevDist = first.location;
        double prevSpeed = first.speedLimit;
        for (SpeedLimit limit  : getSpeedLimits(traj.getStartLocation(), traj.getEndLocation())) {
          planManeuvers(traj, prevDist, limit.location, prevSpeed, limit.speedLimit);
          prevDist = limit.location;
          prevSpeed = limit.speedLimit;
        }

        // Hold the last speed limit until the end of the trajectory
        SpeedLimit last = trajLimits.get(trajLimits.size() - 1);
        planManeuvers(traj, last.location, traj.getEndLocation(), last.speedLimit, last.speedLimit);
      }
    }
  }
}
