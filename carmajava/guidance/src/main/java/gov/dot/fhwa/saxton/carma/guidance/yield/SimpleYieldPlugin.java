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

// The Yield Plugin exists to resolve vehicle trajectory conflicts, or in English, to handle the
// situation where two vehicles, based on existing trajectories or on trajectories that are still
// under consideration, will collide at some point in the near future.  A decision is made at a
// higher level (based on urgencies, priorities, or other factors) to determine which of the two
// vehicles should replan its trajectory, so when the Yield plugin is "activated" on a vehicle,
// it doesn't have to make that determination.  It knows that it was called, and therefore its
// responsibility is to change its vehicle's trajectory to avoid the collision.

// Version 1.0.0 will work as follows.  It will be passed the vehicle trajectory (or somehow gain
// access to it) along with a collision "point" or region (that could have a starting and ending
// downtrack distance (d) and time (t)).  It will request a new blank trajectory and it will
// calculate the downtrack distance (d - Delta) where it will be "safe" from the collision,
// effectively arriving behind the other vehicle instead of colliding with it.  To accomplish this
// it will loop through the maneuvers (Mi) in the "old" vehicle trajectory, copying each one to the
// new trajectory until it finds the maneuver, Mc, that would cause the collision.  At this point it
// allocates a new, blank maneuver.  It makes whatever function call or calls that are necessary to
// populate the maneuver with a path(?) that has the same starting information as Mc, but ends at
// d - Delta instead of at d.  For now, that's it.

// ??? -- This partially filled trajectory is returned to the planner or arbitrator which does
// something TBD.  Alternatively, it calls ACC(?) to fill out the rest of the trajectory and returns
// that as a complete new trajectory that avoids the collision.  This may be sufficient for initial
// testing.  Alternatively, (or for version 1.1.0), this new trajectory can be run through collision
// checking again, and if another collision is found (at a higher d value than before), Yield is
// called again.  If each time Yield is called for this single trajectory, the collision is further
// downtrack (d is always getting larger, and d-delta is always located in a new Maneuver), then
// this process will terminate.  We should be careful to avoid loops where even though we move further
// downtrack, we are still redoing the same maneuver over and over again.

// Other future implementations may include having Yield planning the entire new trajectory.  It would
// process everything through Mc as before, but instead of returning control back to the arbitrator,
// planner, or guidance (???), it would recursively (or maybe iteratively) call the planner/arbitrator
// to fill in the rest of the trajectory.  This is all TBD later after v1.0.0 is build and tested.

package gov.dot.fhwa.saxton.carma.guidance.yield;

import cav_msgs.NewPlan;
import cav_msgs.PlanStatus;
import cav_msgs.PlanType;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityPath;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityPathHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestHandler;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import org.ros.exception.RosRuntimeException;

public class SimpleYieldPlugin extends AbstractPlugin
        implements IStrategicPlugin, MobilityRequestHandler, MobilityPathHandler {

    /* Constants */
    private static final String YIELD_STRATEGY = "carma/yield";
	private static final long SLEEP_DURATION = 10000;
    private double minConflictAvoidanceTimegap = 4.0;
    private double maxYieldAccelAuthority = 2.0;

    private class ReplanData {
        Trajectory trajectoryToReplan;
        ConflictSpace conflictSpace;
        String planId;
    }

    protected int state_ = 0;
    protected AtomicReference<Optional<ReplanData>> replanData = new AtomicReference<>(Optional.empty());

    public SimpleYieldPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Simple Yield Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
        ParameterSource params = pluginServiceLocator.getParameterSource();
        minConflictAvoidanceTimegap = params.getDouble("~min_conflict_avoidance_timegap");
        maxYieldAccelAuthority = params.getDouble("~max_acceleration_capability") * params.getDouble("~max_yield_accel_authority");
        pluginServiceLocator.getMobilityRouter().registerMobilityPathHandler(YIELD_STRATEGY, this);
        pluginServiceLocator.getMobilityRouter().registerMobilityRequestHandler(YIELD_STRATEGY, this);
    }

    @Override
    public void onResume() {
        setAvailability(true);
        log.info("Yield plugin resumed");
    }

    // TODO: Main documentation goes here
    @Override
    public void loop() throws InterruptedException {
        Thread.sleep(SLEEP_DURATION);
        log.info("Yield main loop");
    }

    @Override
    public void onSuspend() {
        setAvailability(false);
        log.info("Yield plugin suspended");
    }

    @Override
    public void onTerminate() {
        log.info("Yield plugin terminated");
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory trajectory, double expectedEntrySpeed) {
        Optional<ReplanData> replanData = this.replanData.get();
        this.replanData.set(Optional.empty());

        if (!replanData.isPresent()) {
            // Nothing to do, no conflict to handle yet
            return new TrajectoryPlanningResponse();
        }

        // Else, assume we've caused this replan and handle the conflict as best we know how
        ConflictSpace conflict = replanData.get().conflictSpace;
        String planId = replanData.get().planId;
        Trajectory oldTraj = replanData.get().trajectoryToReplan;
        log.info(String.format(
                "Yield plugin replanning trajectory [%.02f, %.02f) due to conflicts at [%.02f, %.02f]m t=[%.02f, %.02f] with plan=%s from vehicle=%s",
                trajectory.getStartLocation(), trajectory.getEndLocation(), conflict.getStartDowntrack(),
                conflict.getEndDowntrack(), conflict.getStartTime(), conflict.getEndTime(), planId, "UNKNOWN"));

        // Find the good maneuvers
        List<LongitudinalManeuver> lonMvrs = new ArrayList<>();

        // Load the stack of maneuvers we'll iterate over
        for (LongitudinalManeuver m : oldTraj.getLongitudinalManeuvers()) {
            if (m.getEndDistance() < conflict.getStartDowntrack()) {
                lonMvrs.add(m);
            }
        }

        List<RoutePointStamped> oldPathPrediction = pluginServiceLocator.getTrajectoryConverter().convertToPath(oldTraj);

        double conflictAvoidanceStartDist = 0.0;
        double conflictAvoidanceStartSpeed = 0.0;
        double requiredAcceleration = 0.0;
        double conflictAvoidanceStartTime = 0.0;
        boolean solved = false;
        while (!lonMvrs.isEmpty() && !solved) {
            LongitudinalManeuver mvr = lonMvrs.remove(lonMvrs.size() - 1);
            conflictAvoidanceStartDist = mvr.getEndDistance();
            conflictAvoidanceStartSpeed = mvr.getTargetSpeed();
            
            conflictAvoidanceStartTime = oldPathPrediction.get(0).getStamp();

            for (RoutePointStamped pt : oldPathPrediction) {
                if (pt.getDowntrack() < conflictAvoidanceStartDist) {
                    conflictAvoidanceStartTime = pt.getStamp();
                }
            }

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist;
            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2); 

            if (requiredAcceleration <= maxYieldAccelAuthority) {
                solved = true;
            }
        }

        if (!solved) {
            // Try one last time from the start of the trajectory
            conflictAvoidanceStartDist = trajectory.getStartLocation();
            conflictAvoidanceStartSpeed = expectedEntrySpeed;
            
            conflictAvoidanceStartTime = System.currentTimeMillis();

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist;
            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2); 
        }

        if (requiredAcceleration > maxYieldAccelAuthority) {
            // Nothing we can do, throw control to driver
            throw new RosRuntimeException(String.format("Yield plugin unable to solve conflict at [%.02f, %.02f]m within acceleration constraints maxYieldAccelAuthority=%.02m/s/s, requiredAccel=%.02fm/s/s",
            conflict.getStartDowntrack(), conflict.getEndDowntrack(), maxYieldAccelAuthority, requiredAcceleration));
        }


        // We solved it, implement solution
        for (LongitudinalManeuver mvr : lonMvrs) {
            trajectory.addManeuver(mvr);
        }

        double finalVelocity = conflictAvoidanceStartSpeed + 0.5 * requiredAcceleration;

        SlowDown conflictAvoidanceMvr = new SlowDown(this);
        conflictAvoidanceMvr.setSpeeds(conflictAvoidanceStartSpeed, finalVelocity);
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        planner.planManeuver(conflictAvoidanceMvr, conflictAvoidanceStartDist);
        trajectory.addManeuver(conflictAvoidanceMvr);
        if (conflictAvoidanceMvr.getEndDistance() < conflict.getEndDowntrack()) {
            SteadySpeed steady = new SteadySpeed(this);
            steady.setSpeeds(finalVelocity, finalVelocity);
            planner.planManeuver(steady, conflictAvoidanceMvr.getEndDistance(), conflict.getEndDowntrack());
            trajectory.addManeuver(steady);
        }

        return new TrajectoryPlanningResponse();
    }

    private void updateReplanData(ConflictSpace conflictSpace, String planId) {
        ReplanData replanData = new ReplanData();
        replanData.trajectoryToReplan = pluginServiceLocator.getArbitratorService().getCurrentTrajectory();
        replanData.conflictSpace = conflictSpace;
        replanData.planId = planId;
        this.replanData.set(Optional.of(replanData));
    }

    private void handleConflictNotification(String planId, String conflictingVehicleId, ConflictSpace conflictSpace) {
        updateReplanData(conflictSpace, planId);
        pluginServiceLocator.getArbitratorService().requestNewPlan();
    }

    @Override
    public MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict,
            ConflictSpace conflictSpace) {
        handleConflictNotification(msg.getHeader().getPlanId(), msg.getHeader().getSenderId(), conflictSpace);
        return MobilityRequestResponse.ACK;
    }

    @Override
    public void handleMobilityPathMessageWithConflict(MobilityPath msg, boolean hasConflict,
            ConflictSpace conflictSpace) {
        handleConflictNotification(msg.getHeader().getPlanId(), msg.getHeader().getSenderId(), conflictSpace);
    }
}
