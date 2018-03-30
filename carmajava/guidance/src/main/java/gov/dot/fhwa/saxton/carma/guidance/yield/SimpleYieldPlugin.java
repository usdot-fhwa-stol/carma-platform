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
    private double vehicleResponseLag = 1.4;

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
        minConflictAvoidanceTimegap = params.getDouble("~min_conflict_avoidance_timegap", 4.0);
        maxYieldAccelAuthority = params.getDouble("~max_acceleration_capability", 2.5)
                * params.getDouble("~max_yield_accel_authority", 0.8);
        maxYieldAccelAuthority = params.getDouble("~vehicle_response_lag", 1.4);
        pluginServiceLocator.getMobilityRouter().registerMobilityPathHandler(YIELD_STRATEGY, this);
        pluginServiceLocator.getMobilityRouter().registerMobilityRequestHandler(YIELD_STRATEGY, this);

        log.info(String.format(
                "Yield plugin inited with maxYieldAccelAuthority =%.02f, minConflictAvoidanceTimegap = %.02f",
                minConflictAvoidanceTimegap, maxYieldAccelAuthority));
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

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist - (conflictAvoidanceStartSpeed * vehicleResponseLag);
            if (spaceAvailableForConflictAvoidance <= 0) {
                // Definitely wont work skip to next iteration
                continue;
            }

            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime - vehicleResponseLag;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2);

            if (Math.abs(requiredAcceleration) <= maxYieldAccelAuthority) {
                solved = true;
            }
        }

        if (!solved) {
            // Try one last time from the start of the trajectory
            conflictAvoidanceStartDist = trajectory.getStartLocation();
            conflictAvoidanceStartSpeed = expectedEntrySpeed;

            conflictAvoidanceStartTime = System.currentTimeMillis();

            double spaceAvailableForConflictAvoidance = conflict.getStartDowntrack() - conflictAvoidanceStartDist - (vehicleResponseLag * expectedEntrySpeed);
            double timeAvailableForConflictAvoidance = conflict.getStartTime() - conflictAvoidanceStartTime - vehicleResponseLag;

            // Rename params for conciseness
            double d = spaceAvailableForConflictAvoidance;
            double v = conflictAvoidanceStartSpeed;
            double t = timeAvailableForConflictAvoidance;
            double epsilon = minConflictAvoidanceTimegap;
            requiredAcceleration = 2 * (d - (v * (t + epsilon))) / Math.pow(t + epsilon, 2);
        }

        if (Math.abs(requiredAcceleration) > maxYieldAccelAuthority) {
            // Nothing we can do, throw control to driver
            throw new RosRuntimeException(String.format(
                    "Yield plugin unable to solve conflict at [%.02f, %.02f]m within acceleration constraints maxYieldAccelAuthority=%.02m/s/s, requiredAccel=%.02fm/s/s",
                    conflict.getStartDowntrack(), conflict.getEndDowntrack(), maxYieldAccelAuthority,
                    requiredAcceleration));
        }

        // We solved it, implement solution
        for (LongitudinalManeuver mvr : lonMvrs) {
            trajectory.addManeuver(mvr);
        }

        double finalVelocity = conflictAvoidanceStartSpeed +  requiredAcceleration * (conflict.getStartTime() - conflictAvoidanceStartTime);

        SlowDown conflictAvoidanceMvr = new SlowDown(this);
        conflictAvoidanceMvr.setSpeeds(conflictAvoidanceStartSpeed, finalVelocity);
        ManeuverPlanner planner = pluginServiceLocator.getManeuverPlanner();
        planner.planManeuver(conflictAvoidanceMvr, conflictAvoidanceStartDist);
        trajectory.addManeuver(conflictAvoidanceMvr);
        if (conflictAvoidanceMvr.getEndDistance() <= conflict.getEndDowntrack()) {
            SteadySpeed steady = new SteadySpeed(this);
            steady.setSpeeds(finalVelocity, finalVelocity);
            planner.planManeuver(steady, conflictAvoidanceMvr.getEndDistance(), conflict.getEndDowntrack());
            trajectory.addManeuver(steady);
        }

        // Copy the unchanged lateral maneuvers
        double latMvrsEnd = Double.POSITIVE_INFINITY;
        for (LateralManeuver mvr : oldTraj.getLateralManeuvers()) {
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
