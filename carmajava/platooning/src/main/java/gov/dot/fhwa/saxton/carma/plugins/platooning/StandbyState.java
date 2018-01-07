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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.SortedSet;

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.AlgorithmFlags;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to LeaderState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the pulgin will not insert any maneuvers into a trajectory and ignore all negotiation messages.
 */
public class StandbyState implements IPlatooningState {

    @Override
    public TrajectoryPlanningResponse planTrajectory(PlatooningPlugin plugin, ILogger log,
            PluginServiceLocator pluginServiceLocator, Trajectory traj, double expectedEntrySpeed) {
        RouteService routeService = pluginServiceLocator.getRouteService();
        SortedSet<AlgorithmFlags> flags = routeService.getAlgorithmFlagsInRange(traj.getStartLocation(), traj.getEndLocation());
        flags.add(routeService.getAlgorithmFlagsAtLocation(traj.getEndLocation()));
        for(AlgorithmFlags flag : flags) {
            if(!flag.getDisabledAlgorithms().contains(plugin.PLATOONING_FLAG)) {
                plugin.setState(new LeaderState());
                return plugin.planTrajectory(traj, expectedEntrySpeed);
            }
        }
        return new TrajectoryPlanningResponse();
    }

    @Override
    public void onReceiveNegotiationRequest(PlatooningPlugin plugin, ILogger log,
            PluginServiceLocator pluginServiceLocator, String plan) {
        // NO-OP
    }

    @Override
    public String toString() {
        return "StandbyState";
    }
}
