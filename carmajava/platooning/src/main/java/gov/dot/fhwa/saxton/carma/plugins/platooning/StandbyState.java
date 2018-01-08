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

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to LeaderState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the pulgin will not insert any maneuvers into a trajectory and ignore all negotiation messages.
 */
public class StandbyState implements IPlatooningState {

    protected PlatooningPlugin plugin_;
    protected ILogger log_;
    protected PluginServiceLocator pluginServiceLocator_;
    
    public StandbyState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        plugin_ = plugin;
        log_ = log;
        pluginServiceLocator_ = pluginServiceLocator;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        if(pluginServiceLocator_.getRouteService().hasFlagInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            //TODO it may need to send out some mobility messages when the transition happened
            plugin_.setState(new LeaderState(plugin_, log_, pluginServiceLocator_));
            return plugin_.planTrajectory(traj, expectedEntrySpeed);
        }
        return new TrajectoryPlanningResponse();
    }

    @Override
    public void onReceiveNegotiationRequest(String plan) {
        // NO-OP
    }

    @Override
    public String toString() {
        return "StandbyState";
    }
}
