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
// responsibility is to change its vehicle's trajectory to avoid the collision.  There are some
// exceptions to this, such as during initialization time when the plugin registers itself, or
// when the planning function is called while there is no valid trajectory stored.

// Version 1.0.0 will work as follows.  It will be passed the vehicle trajectory (or somehow gain
// access to it) along with a collision "point" or region (that could have a starting and ending
// downtrack distance (d) and time (t)).  It will request a new blank trajectory and it will
// calculate the downtrack distance (d - Delta) where it will be "safe" from the collision,
// effectively trying to arrive behind the other vehicle instead of colliding with it.  To accomplish
// this it loops through the maneuvers (Mi) in the "old" vehicle trajectory, copying each one to the
// new trajectory until it finds the maneuver, Mc, that would cause the collision.  At this point it
// allocates a new, blank maneuver.  It makes whatever function call or calls that are necessary to
// populate the maneuver with a path(?) that has the same starting information as Mc, but ends at
// at downtrack location d - Delta instead of at d.  For now, that's it.  Once we get this working,
// it will be relatively easy to add more complex decision-making to support more situations, but for
// the upcoming demo, it should work fine.  If we are worried that it will sense a collision with a
// vehicle that leads a non-empty platoon, or with a vehicle that happens to have a car close behind
// it, we will have to be careful to increase Delta to account for this, or to collision-check our
// solution, see that we are now colliding with the car behind it, and repeat (as necessary).

// Kyle wrote v1.0.0 on 30-March-2018.
// Barry merged his skeletal version, primarily documentation (these paragraphs) and comments, with
//     v1.0.0 to create v1.0.1 on 3-April-2008.  This "paragraph" added, minor edits to others.
// Barry will confirm that the algorithm and the Math are correct to create v1.0.2 or v1.1.0 depending
//     on the extent of the changes.
// Barry will resolve all "TODOs" (an Intermediate or Minor version change as above).
// ... Implementation test logs exist from a run where Lane Change failed ... Unclear if Yield worked
// ... Confirm that one of the above versions executes correctly ... Force situation, Check logs
// ... Determine requirements and acceptance criteria for v2.0.0 ("More Robust") ... Implement & Test

// ??? -- This partially filled trajectory is returned to the planner or arbitrator which does
// something TBD.  Alternatively, it calls ACC(?) to fill out the rest of the trajectory and returns
// that as a complete new trajectory that avoids the collision.  This may be sufficient for initial
// testing.  Alternatively, (or for version 1.1.0), this new trajectory can be run through collision
// checking again, and if another collision is found (at a higher d value than before), Yield is
// called again.  If each time Yield is called for this single trajectory, the collision is further
// downtrack (d is always getting larger, and d - Delta is always located in a new Maneuver), then
// this process will terminate.  We should be careful to avoid loops where even though we move further
// downtrack, we are still redoing the same maneuver over and over again.  We should remember to check
// for any possible infinite loop, no matter how unlikely, and incorporate a maximum recursion depth
// constant to avoid it.

// Other future implementations may include having Yield plan the entire new trajectory.  It would
// process everything through Mc as before, but instead of returning control back to the arbitrator,
// planner, or guidance (???), it would recursively (or maybe iteratively) call the planner/arbitrator
// to fill in the rest of the trajectory.

package gov.dot.fhwa.saxton.carma.guidance.yield;

import cav_msgs.MobilityRequest;
import cav_msgs.MobilityPath;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictSpace;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LaneKeeping;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SlowDown;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SpeedUp;
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

public class YieldPlugin extends AbstractPlugin
        implements IStrategicPlugin, MobilityRequestHandler, MobilityPathHandler {

    /* Constants */
    private static final String YIELD_STRATEGY = "carma/yield";
    private static final long SLEEP_DURATION = 10000; // Milliseconds: 10000 = 10 Seconds
    private double minConflictAvoidanceTimegap = 4.0; // Seconds
    private double maxYieldAccelAuthority = 2.0; // Meters per Second per Second
    private double vehicleResponseLag = 1.4; // Seconds
    private static final double SPEED_EPSILON = 0.01;

    private class ReplanData {
        Trajectory trajectoryToReplan;
        ConflictSpace conflictSpace;
        String planId;
    }

    // For now, there are two states. One in which there is no trajectory stored,
    // and one in which there is. Initially we are
    // in the former state - call it state 0. That is also the start state. To
    // transition out of this state, we must receive a
    // call to handleMobilityPathMessageWithConflict() or
    // handleMobilityRequestMessage(). In either case we are passed a
    // trajectory (and a collision point?, and other information?). We store this
    // trajectory (and other information) in class
    // variables, and if successful, transition from state 0 to state 1 by changing
    // the value of the class variable state_ to 1.

    // To transition back to state 0, we must receive a call to planTrajectory(). If
    // planTrajectory() is called while in state 0,
    // nothing happens. It returns. If it is called in state 1, then this is where
    // the real work is done. As described above, a
    // new trajectory is created, replacing the old one, where the collision is
    // avoided by having our vehicle arrive behind the
    // collision point or region at the collision time (the details of which are
    // TBD).

    /* Variables */
    protected int state_ = 0;
    protected AtomicReference<Optional<ReplanData>> replanData = new AtomicReference<>(Optional.empty());

    // Constructor
    public YieldPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("Yield Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(1);
    }

    @Override
    public void onInitialize() {
        ParameterSource params = pluginServiceLocator.getParameterSource();
        minConflictAvoidanceTimegap = params.getDouble("~min_conflict_avoidance_timegap", 4.0);
        maxYieldAccelAuthority = params.getDouble("~vehicle_acceleration_limit", 2.5)
                * params.getDouble("~max_yield_accel_authority", 0.8);
        vehicleResponseLag = params.getDouble("~vehicle_response_lag", 1.4);

        log.info(String.format(
                "Yield plugin inited with maxYieldAccelAuthority =%.02f, minConflictAvoidanceTimegap = %.02f",
                maxYieldAccelAuthority, minConflictAvoidanceTimegap));
    }

    @Override
    public void onResume() {
        setAvailability(true);
        pluginServiceLocator.getMobilityRouter().registerMobilityPathHandler(YIELD_STRATEGY, this);
        pluginServiceLocator.getMobilityRouter().registerMobilityRequestHandler(YIELD_STRATEGY, this);
        log.info("Yield plugin resumed");
    }

    // Main Loop
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

        // Else, assume we've caused this replan and handle the conflict as best we know
        // how
        ConflictSpace conflict = replanData.get().conflictSpace;
        String planId = replanData.get().planId;
        Trajectory oldTraj = replanData.get().trajectoryToReplan;
        log.info(String.format(
                "Yield plugin replanning trajectory [%.02f, %.02f) due to conflicts at [%.02f, %.02f]m t=[%.02f, %.02f] with plan=%s",
                trajectory.getStartLocation(), trajectory.getEndLocation(), conflict.getStartDowntrack(),
                conflict.getEndDowntrack(), conflict.getStartTime(), conflict.getEndTime(), planId));

        List<LongitudinalManeuver> lonMvrs = new ArrayList<>();

        // Load the stack of maneuvers we'll iterate over
        for (LongitudinalManeuver m : oldTraj.getLongitudinalManeuvers()) {
            if (m.getEndDistance() < conflict.getStartDowntrack()) {
                lonMvrs.add(m);
            }
        }

        List<RoutePointStamped> oldPathPrediction = pluginServiceLocator.getTrajectoryConverter()
                .convertToPath(oldTraj);

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

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist
                    - (conflictAvoidanceStartSpeed * vehicleResponseLag);
            if (spaceAvailableForConflictAvoidance <= 0) {
                // Definitely wont work skip to next iteration
                continue;
            }

            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime
                    - vehicleResponseLag;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2);

            log.debug(String.format("Evaluating candidate solution: d=%.02f, v=%.02f, t=%.02f, epsilon=%.02f, a=%.02f",
                    d, v, t, epsilon, requiredAcceleration));

            if (Math.abs(requiredAcceleration) <= maxYieldAccelAuthority) {
                log.debug("Candidate solution acceptable.");
                solved = true;
            } else {
                log.debug("Candidate solution unacceptable, iterating...");
            }
        }

        if (!solved) {
            // Try one last time from the start of the trajectory
            conflictAvoidanceStartDist = trajectory.getStartLocation();
            conflictAvoidanceStartSpeed = expectedEntrySpeed;

            conflictAvoidanceStartTime = System.currentTimeMillis() / 1000.0;

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist
                    - (vehicleResponseLag * expectedEntrySpeed);
            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime
                    - vehicleResponseLag;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2);
            log.debug(String.format(
                    "Evaluating last effort solution from start of trajectory: d=%.02f, v=%.02f, t=%.02f, epsilon=%.02f, a=%.02f",
                    d, v, t, epsilon, requiredAcceleration));
        }

        if (Math.abs(requiredAcceleration) > maxYieldAccelAuthority) {
            log.error("Overall solution unacceptable, throwing error!");
            // Nothing we can do, throw control to driver
            throw new RosRuntimeException(String.format(
                    "Yield plugin unable to solve conflict at [%.02f, %.02f]m within acceleration constraints maxYieldAccelAuthority=%.02fm/s/s, requiredAccel=%.02fm/s/s",
                    conflict.getStartDowntrack(), conflict.getEndDowntrack(), maxYieldAccelAuthority,
                    requiredAcceleration)); // TODO: Improve robustness of algo to return NACK if is request or look for
                                            // alternative type of soln (lane change, etc.)
        } else {
            log.info("Overall solution within acceleration constraints!");
        }

        // We solved it, implement solution
        for (LongitudinalManeuver mvr : lonMvrs) {
            log.info(String.format("Yield plugin keeping longitudinal maneuver from [%.02f, %.02f)",
                    mvr.getStartDistance(), mvr.getEndDistance()));
            trajectory.addManeuver(mvr);
        }

        double finalVelocity = conflictAvoidanceStartSpeed
                + requiredAcceleration * (conflict.getStartTime() - conflictAvoidanceStartTime);

        log.debug(String.format(
                "conflictAvoidanceStartSpeed=%.02f, requiredAcceleration=%.02f, finalVelocity=%.02f, conflictAvoidanceStartTime=%.02f, conflictStartTime=%.02f",
                conflictAvoidanceStartSpeed, requiredAcceleration, finalVelocity, conflictAvoidanceStartTime,
                conflict.getStartTime()));

        LongitudinalManeuver conflictAvoidanceMvr;
        // Use delta-v because even small accel changes may be significant over large distances or time
        double deltaV = finalVelocity - conflictAvoidanceStartSpeed;
        if (deltaV < -SPEED_EPSILON) {
            // Plan a slow-down maneuver
            conflictAvoidanceMvr = new SlowDown(this);
            conflictAvoidanceMvr.setSpeeds(conflictAvoidanceStartSpeed, finalVelocity);
        } else if (deltaV > SPEED_EPSILON) {
            conflictAvoidanceMvr = new SpeedUp(this);
            conflictAvoidanceMvr.setSpeeds(conflictAvoidanceStartSpeed, finalVelocity);
        } else {
            conflictAvoidanceMvr = new SteadySpeed(this);
            // Ensure we don't cause an arithmetic error
            conflictAvoidanceMvr.setSpeeds(conflictAvoidanceStartSpeed, conflictAvoidanceStartSpeed);
        }

        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        planner.planManeuver(conflictAvoidanceMvr, conflictAvoidanceStartDist);
        log.info(String.format("Yield plugin planned %s maneuver from [%.02f, %.02f) with speeds [%.02f, %.02f]",
                conflictAvoidanceMvr.getClass().getSimpleName(), conflictAvoidanceStartDist, conflictAvoidanceMvr.getEndDistance(), conflictAvoidanceMvr.getStartSpeed(),
                conflictAvoidanceMvr.getTargetSpeed()));
        trajectory.addManeuver(conflictAvoidanceMvr);
        if (conflictAvoidanceMvr.getEndDistance() <= conflict.getEndDowntrack()) {
            log.info(String.format("Yield plugin planning steady speed maneuver from [%.02f, %.02f) @ %.02f m/s",
                    conflictAvoidanceMvr.getEndDistance(), conflict.getEndDowntrack(), finalVelocity));
            SteadySpeed steady = new SteadySpeed(this);
            steady.setSpeeds(finalVelocity, finalVelocity);
            planner.planManeuver(steady, conflictAvoidanceMvr.getEndDistance(), conflict.getEndDowntrack());
            trajectory.addManeuver(steady);
        }

        // Copy the unchanged lateral maneuvers
        double latMvrsEnd = Double.POSITIVE_INFINITY;
        for (LateralManeuver mvr : oldTraj.getLateralManeuvers()) {
            log.info(String.format("Yield plugin keeping lateral maneuver from [%.02f, %.02f)", mvr.getStartDistance(),
                    mvr.getEndDistance()));
            if (mvr.getEndDistance() < conflictAvoidanceStartDist) {
                trajectory.addManeuver(mvr);
                latMvrsEnd = mvr.getEndDistance();
            }
        }

        // Backfill the lateral maneuvers up to the end of the conflict
        if (latMvrsEnd < conflict.getEndDowntrack()) {
            LaneKeeping laneKeeping = new LaneKeeping(this);
            IGuidanceCommands gc = pluginServiceLocator.getManeuverPlanner().getGuidanceCommands();
            IManeuverInputs mi = pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
            laneKeeping.planToTargetDistance(mi, gc, latMvrsEnd, conflict.getEndDowntrack());
            log.info(String.format("Yield plugin backfilling with lane keeping maneuvers from [%.02f, %.02f)",
                    latMvrsEnd, conflict.getEndDowntrack()));
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

    @Override // Primary Entry Point 1/2
    public MobilityRequestResponse handleMobilityRequestMessage(MobilityRequest msg, boolean hasConflict,
            ConflictSpace conflictSpace) {
        handleConflictNotification(msg.getHeader().getPlanId(), msg.getHeader().getSenderId(), conflictSpace);
        return MobilityRequestResponse.ACK;
    }

    @Override // Primary Entry Point 2/2
    public void handleMobilityPathMessageWithConflict(MobilityPath msg, boolean hasConflict,
            ConflictSpace conflictSpace) {
        if (msg != null) {
            handleConflictNotification(msg.getHeader().getPlanId(), msg.getHeader().getSenderId(), conflictSpace);
        } else {
            handleConflictNotification("VehicleAwareness", "UNSPECIFIED", conflictSpace);
        }
    }
}

// END File YieldPlugin.java
