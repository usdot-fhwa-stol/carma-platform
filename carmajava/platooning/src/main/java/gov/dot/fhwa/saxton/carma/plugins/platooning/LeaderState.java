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

public class LeaderState implements PlatooningState {

    @Override
    public TrajectoryPlanningResponse planTrajectory(PlatooningPlugin plugin, ILogger log,
            PluginServiceLocator pluginServiceLocator, Trajectory traj, double expectedEntrySpeed) {
        RouteService routeService = pluginServiceLocator.getRouteService();
        SortedSet<AlgorithmFlags> flags = routeService.getAlgorithmFlagsInRange(traj.getStartLocation(), traj.getEndLocation());
        flags.add(routeService.getAlgorithmFlagsAtLocation(traj.getEndLocation()));
        Object[] flagArray = flags.toArray();
        for(int i = 0; i < flagArray.length; i++) {
            if(!((AlgorithmFlags) flagArray[i]).getDisabledAlgorithms().contains(plugin.PLATOONING_FLAG)) {
                break;
            }
            if(i == flagArray.length - 1) {
                plugin.setState(new StandbyState());
                return plugin.planTrajectory(traj, expectedEntrySpeed);
            }
        }
        return new TrajectoryPlanningResponse();
    }

    @Override
    public void onReceiveNegotiationRequest(PlatooningPlugin plugin, ILogger log,
            PluginServiceLocator pluginServiceLocator, String plan) {
        // TODO Auto-generated method stub
        
    }
    
    @Override
    public String toString() {
        return "LeaderState";
    }
}
