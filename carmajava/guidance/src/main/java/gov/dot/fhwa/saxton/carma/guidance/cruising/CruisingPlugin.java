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

package gov.dot.fhwa.saxton.carma.guidance.cruising;

import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.SortedSet;

/**
 * Cruising Plugin
 * </p>
 * Implements the basic behavior of commanding the speed limit (as specified by RouteManager)
 * at each route segment.
 */
public class CruisingPlugin extends AbstractPlugin implements IStrategicPlugin {
    
  protected double maxAccel_;
  protected double cruisingTargetMultiplier_;
  protected static final double DISTANCE_EPSILON = 0.0001;
  protected static final double SPEED_EPSILON = 0.0001;
  protected RouteService routeService;

  protected class TrajectorySegment {
    double startLocation;
    double endLocation;
    double startSpeed;
    
    @Override
    public String toString() {
        return "Gap: [startLocation = " + startLocation + "; endLocation = "
               + endLocation + "; startSpeed = " + startSpeed + "]";
    }
  }

  public CruisingPlugin(PluginServiceLocator psl) {
    super(psl);
    // !!!!!!!!!!
    // Any modification on this plugin should result in a change on its version ID.
    // !!!!!!!!!!
    version.setName("Cruising Plugin");
    version.setMajorRevision(1);
    version.setIntermediateRevision(0);
    version.setMinorRevision(2);
  }

  @Override
  public void onInitialize() {
    log.info("Cruisng plugin initializing...");
    maxAccel_ = pluginServiceLocator.getParameterSource().getDouble("~vehicle_acceleration_limit", 2.5);
    cruisingTargetMultiplier_ = pluginServiceLocator.getParameterSource().getDouble("~cruising_target_multiplier", 1.0);
    cruisingTargetMultiplier_ = Math.max(0.0, cruisingTargetMultiplier_);
    cruisingTargetMultiplier_ = Math.min(1.0, cruisingTargetMultiplier_);
    routeService = pluginServiceLocator.getRouteService();
    log.info("Cruising plugin initialized.");
  }

  @Override
  public void loop() {
    try {
      Thread.sleep(5000);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
    }
  }

  @Override
  public void onResume() {
    setAvailability(true);
  }

  @Override
  public void onSuspend() {
    setAvailability(false);
  }

  @Override
  public void onTerminate() {
    // NO-OP
  }

  protected boolean fpEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

    // TODO this method is currently looking for gaps in longitudinal maneuvers but
    // it needs to expand to lateral in the future
    protected List<TrajectorySegment> findTrajectoryGaps(Trajectory traj, double trajStartSpeed) {

        // Get the end location of this trajectory excluded complex maneuver
        double endLocation = traj.getComplexManeuver() == null ? traj.getEndLocation() : traj.getComplexManeuver().getStartDistance();
        // Get the list of sorted longitudinal maneuvers to calculate longitudinal plan gaps
        List<LongitudinalManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
        longitudinalManeuvers.sort((o1, o2) -> Double.compare(o1.getStartDistance(), o2.getStartDistance()));
        List<TrajectorySegment> gaps = new ArrayList<>();
        double lastManeuverEndLocation = traj.getStartLocation();
        double lastManeuverEndSpeed = trajStartSpeed;
        for (LongitudinalManeuver lm : longitudinalManeuvers) {
            if (!fpEquals(lastManeuverEndLocation, lm.getStartDistance(), DISTANCE_EPSILON)) {
             // create trajectory seg and update last maneuver end location/speed
                TrajectorySegment seg = new TrajectorySegment();
                seg.startLocation = lastManeuverEndLocation;
                seg.endLocation = lm.getStartDistance();
                seg.startSpeed = lastManeuverEndSpeed;
                gaps.add(seg);
            }
            lastManeuverEndLocation = lm.getEndDistance();
            lastManeuverEndSpeed = lm.getTargetSpeed();
        }

        if (!fpEquals(lastManeuverEndLocation, endLocation, DISTANCE_EPSILON)) {
            TrajectorySegment seg = new TrajectorySegment();
            seg.startLocation = lastManeuverEndLocation;
            seg.endLocation = endLocation;
            seg.startSpeed = lastManeuverEndSpeed;
            gaps.add(seg);
        }
        
        log.info("Found " + gaps.size() + " empty gaps in trajectory ["
                 + traj.getStartLocation() + ", " + traj.getEndLocation() + "]");
        for(TrajectorySegment seg : gaps) {
            log.info(seg.toString());
        }
        return gaps;
    }

  /**
   * It tries to plan a maneuver with give startDist, endDist, startSpeed and endSpeed.
   * The return value is the actual endSpeed.
   */
  protected double planManeuvers(Trajectory t, double startDist, double endDist, double startSpeed, double endSpeed) {
    ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
    SimpleManeuverFactory maneuverFactory = new SimpleManeuverFactory(this);
    log.info(String.format("Trying to plan maneuver {start=%.2f,end=%.2f,startSpeed=%.2f,endSpeed=%.2f}", startDist, endDist, startSpeed, endSpeed));
    double maneuverEnd = startDist;
    double adjustedEndSpeed = startSpeed;
    LongitudinalManeuver maneuver = maneuverFactory.createManeuver(startSpeed, endSpeed);
    maneuver.setSpeeds(startSpeed, endSpeed);
    maneuver.setMaxAccel(maxAccel_);
    if(planner.canPlan(maneuver, startDist, endDist)) {
        planner.planManeuver(maneuver, startDist);
        if(maneuver instanceof SteadySpeed) {
            ((SteadySpeed) maneuver).overrideEndDistance(endDist);
        }
        if(maneuver.getEndDistance() > endDist) {
            adjustedEndSpeed = planner.planManeuver(maneuver, startDist, endDist);
        } else {
            adjustedEndSpeed = maneuver.getTargetSpeed();
        }
        maneuverEnd = maneuver.getEndDistance();
        t.addManeuver(maneuver);
        log.info(String.format("Planned maneuver from [%.2f, %.2f) m/s over [%.2f, %.2f) m", startSpeed, adjustedEndSpeed, startDist, maneuverEnd));
    }
    
    // Insert a steady speed maneuver to fill whatever's left
    if(maneuverEnd < endDist) {
      LongitudinalManeuver steady = maneuverFactory.createManeuver(adjustedEndSpeed, adjustedEndSpeed);
        steady.setSpeeds(adjustedEndSpeed, adjustedEndSpeed);
        steady.setMaxAccel(maxAccel_);
        planner.planManeuver(steady, maneuverEnd);
        ((SteadySpeed) steady).overrideEndDistance(endDist);
        t.addManeuver(steady);
        log.info(String.format("Planned STEADY-SPEED maneuver at %.2f m/s over [%.2f, %.2f) m", adjustedEndSpeed, maneuverEnd, endDist));
    }
    
    return adjustedEndSpeed;
  }
  
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        for (TrajectorySegment ts : findTrajectoryGaps(traj, expectedEntrySpeed)) {
            planTrajectoryGap(ts, traj);
        }
        return new TrajectoryPlanningResponse();
    }
  
    private void planTrajectoryGap(TrajectorySegment trajSeg, Trajectory traj) {
        // Use RouteService to find all necessary speed limits in this trajectory segment
        SortedSet<SpeedLimit> trajLimits = routeService.getSpeedLimitsInRange(trajSeg.startLocation, trajSeg.endLocation);
        trajLimits.add(routeService.getSpeedLimitAtLocation(trajSeg.endLocation));

        // Merge segments with same speed limits
        List<SpeedLimit> mergedLimits = new LinkedList<SpeedLimit>();
        SpeedLimit limit_buffer = null;
        for (SpeedLimit limit : trajLimits) {
            // Apply the cruising speed percentage to the speed limit
            SpeedLimit followedLimit = new SpeedLimit(limit.getLocation(), limit.getLimit() * cruisingTargetMultiplier_);
            // Merge segments with same speed
            if (limit_buffer == null) {
                limit_buffer = followedLimit;
            } else {
                if (fpEquals(limit_buffer.getLimit(), followedLimit.getLimit(), SPEED_EPSILON)) {
                    limit_buffer.setLocation(followedLimit.getLocation());
                } else {
                    mergedLimits.add(limit_buffer);
                    limit_buffer = followedLimit;
                }
            }
        }
        limit_buffer.setLocation(trajSeg.endLocation);
        mergedLimits.add(limit_buffer);
        log.info("Found " + mergedLimits.size() + " speed limits in " + trajSeg.toString());
        for(SpeedLimit sl : mergedLimits) {
            log.info(sl.toString());
        }
        // Plan trajectory to follow all speed limits in this trajectory segment
        double newManeuverStartSpeed = trajSeg.startSpeed;
        double newManeuverStartLocation = trajSeg.startLocation;
        for (SpeedLimit limit : mergedLimits) {
            newManeuverStartSpeed = planManeuvers(traj, newManeuverStartLocation, limit.getLocation(), newManeuverStartSpeed, limit.getLimit());
            newManeuverStartLocation = limit.getLocation();
        }
    }
}
