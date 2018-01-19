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

package gov.dot.fhwa.saxton.carma.guidance.routefollowing;

import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SimpleManeuverFactory;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.SteadySpeed;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.ITacticalPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManagementService;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.RequiredLane;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.SortedSet;

/**
 * Basic plugin for following necessary lane changes in the currently selected route
 * <p>
 * Delegates planning of the lane change maneuver(s) themselves to the LaneChange plugin
 * by use of the ITacticalPlugin interface.
 */
public class RouteFollowingPlugin extends AbstractPlugin implements IStrategicPlugin {

    private ITacticalPlugin laneChangePlugin;
    private RouteService routeService;

    private static final String LANE_CHANGE_PLUGIN_NAME = "LaneChangePlugin";
    private static final double LANE_CHANGE_RATE_FACTOR = 0.75;
    private static final double LANE_CHANGE_DELAY_FACTOR = 1.5;
    private static final double LANE_CHANGE_SAFETY_FACTOR = 1.5;
    private static final long LONG_SLEEP = 10000;
    private static final double EPSILON = 0.001;

    public RouteFollowingPlugin(PluginServiceLocator psl) {
        super(psl);
        version.setName("Route Following Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
    }

    @Override
    public void onResume() {
        laneChangePlugin = pluginServiceLocator.getPluginManagementService()
                .getTacticalPluginByName(LANE_CHANGE_PLUGIN_NAME);

        if (laneChangePlugin == null) {
            // No lane change plugin was found registered withe the platform, RouteFollowing cannot operate
            throw new RuntimeException(
                    "Route following plugin unable to locate necessary lane change tactical plugin.\nPlease install a lane change plugin.");
        }

        routeService = pluginServiceLocator.getRouteService();
    }

    @Override
    public void loop() throws InterruptedException {
        Thread.sleep(LONG_SLEEP);
    }

    @Override
    public void onSuspend() {

    }

    @Override
    public void onTerminate() {

    }

    private void setLaneChangeParameters(int targetLane, double startSpeedLimit, double endSpeedLimit) {
        // STUB
    }

    private void planLaneKeepingManeuver(double startDist, double endDist) {
        // STUB
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        // TODO: Implement planning logic to handle planning around other lateral maneuvers that may already be planned!
        SortedSet<RequiredLane> requiredLanes = routeService.getRequiredLanesInRange(traj.getStartLocation(),
                traj.getEndLocation());
        requiredLanes.add(routeService.getRequiredLaneAtLocation(traj.getEndLocation())); // Handle the end point of the trajectory

        // Remove duplicates to find out where lane changes must occur
        List<RequiredLane> requiredLaneChanges = new ArrayList<RequiredLane>();
        RequiredLane prev = null;
        for (RequiredLane lane : requiredLanes) {
            if (prev != null && prev.getLaneId() != lane.getLaneId()) {
                requiredLanes.add(lane);
            }

            prev = lane;
        }

        // Ensure that we have enough space after the current trajectory to complete the NEXT lane change
        if (!requiredLaneChanges.isEmpty()) {
            RequiredLane laneChangeAtTrajEnd = requiredLaneChanges.get(requiredLaneChanges.size() - 1);

            if (laneChangeAtTrajEnd.getLocation() > traj.getEndLocation()) {
                double distanceToNextLaneChangeAfterTraj = laneChangeAtTrajEnd.getLocation() - traj.getEndLocation();
                double speedLimit = routeService.getSpeedLimitAtLocation(laneChangeAtTrajEnd.getLocation()).getLimit();
                double distanceRequiredForLaneChange = speedLimit * LANE_CHANGE_DELAY_FACTOR
                        + speedLimit * LANE_CHANGE_RATE_FACTOR + speedLimit * LANE_CHANGE_SAFETY_FACTOR;

                double distanceAvailableForLaneChange = laneChangeAtTrajEnd.getLocation() - traj.getEndLocation();

                if (distanceAvailableForLaneChange < distanceRequiredForLaneChange) {
                    // Tell the arbitrator the next trajectory wont be able to execute the whole lane change, so roll
                    // that maneuver into this one.
                    TrajectoryPlanningResponse requestForLongerTrajectory = new TrajectoryPlanningResponse();
                    requestForLongerTrajectory.requestLongerTrajectory(laneChangeAtTrajEnd.getLocation());
                    return requestForLongerTrajectory;
                }
            }
        }

        // Walk the lane changes required by route and plan maneuvers for each of them.
        for (RequiredLane targetLane : requiredLaneChanges) {
            double speedLimit = routeService.getSpeedLimitAtLocation(targetLane.getLocation()).getLimit();
            double distanceForLaneChange = speedLimit * LANE_CHANGE_DELAY_FACTOR + speedLimit * LANE_CHANGE_RATE_FACTOR
                    + speedLimit * LANE_CHANGE_SAFETY_FACTOR;

            double laneChangeStartLocation = targetLane.getLocation() - distanceForLaneChange;

            if (laneChangeStartLocation < traj.getStartLocation()) {
                // Should not be the case ever but we'll give it a best effort attempt anyway
                laneChangeStartLocation = traj.getStartLocation();
            }

            // Check to see if we're done with the previous lane change
            IManeuver lateralManeuverAtStartLocation = traj.getManeuverAt(laneChangeStartLocation,
                    ManeuverType.LATERAL);
            if (lateralManeuverAtStartLocation != null) {
                // Plan our new maneuver right after that one, giving a best effort attempt
                laneChangeStartLocation = lateralManeuverAtStartLocation.getEndDistance();
            }

            double laneChangeEndLocation = laneChangeStartLocation + distanceForLaneChange;

            // Compute the distance required and command the lane change plugin to plan
            double startSpeedLimit = routeService.getSpeedLimitAtLocation(laneChangeStartLocation).getLimit();
            double endSpeedLimit = routeService.getSpeedLimitAtLocation(laneChangeEndLocation).getLimit();
            setLaneChangeParameters(targetLane.getLaneId(), startSpeedLimit, endSpeedLimit);
            laneChangePlugin.planSubtrajectory(traj, laneChangeStartLocation, laneChangeEndLocation);
        }

        // Now that all lane changes have been planned, backfill with lane keeping
        double windowStart = traj.findEarliestLateralWindowOfSize(EPSILON);
        while (windowStart != -1) {
            IManeuver m = traj.getNextManeuverAfter(windowStart, ManeuverType.LATERAL);
            double windowEnd = (m != null ? m.getStartDistance() : traj.getEndLocation());

            planLaneKeepingManeuver(windowStart, windowEnd);

            windowStart = traj.findEarliestLateralWindowOfSize(EPSILON);
        }

        return new TrajectoryPlanningResponse(); // Success!
    }
}
