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

import java.util.List;

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * The LeaderState is a state when the platooning algorithm is enabled and the host vehicle is not the leader.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to LeaderState when either it cannot maintain the gap with the front vehicle
 * or its leader sends a negotiation message to assign it as the new leader.
 * In this state, the plugin will insert a PlatooningManeuver into the trajectory and control the vehicle.
 */
public class FollowerState implements IPlatooningState {

    protected PlatooningPlugin plugin_;
    protected ILogger log_;
    protected PluginServiceLocator pluginServiceLocator_;
    
    public FollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        plugin_ = plugin;
        log_ = log;
        pluginServiceLocator_ = pluginServiceLocator;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        if(!pluginServiceLocator_.getRouteService().hasFlagInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            //TODO it may need to send out some mobility messages when the transition happened
            plugin_.manager.disablePlatooning();
            plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
            return plugin_.planTrajectory(traj, expectedEntrySpeed);
        }
        // Insert a PlatooningManeuver in the earliest legal window if it is in follower state
        double complexManeuverStartLocation = traj.getStartLocation();
        List<IManeuver> maneuvers = traj.getManeuvers();
        if(!maneuvers.isEmpty()) {
            complexManeuverStartLocation = maneuvers.get(maneuvers.size() - 1).getEndDistance(); 
        }
        double[] window = pluginServiceLocator_.getRouteService().getPluginEnabledWindowInRange(complexManeuverStartLocation, traj.getEndLocation(), plugin_.PLATOONING_FLAG);
        if(window == null || Math.abs(window[1] - window[0]) < plugin_.getMinimumManeuverLength()) {
            log_.warn("Cannot find a legal window to plan a platooning complex maneuver");
        }
        PlatooningManeuver maneuver = new PlatooningManeuver(
                plugin_.commandGenerator,
                pluginServiceLocator_.getManeuverPlanner().getManeuverInputs(),
                pluginServiceLocator_.getManeuverPlanner().getGuidanceCommands(),
                AccStrategyManager.newAccStrategy(),
                window[0], window[1],
                1.0, 100.0);
        boolean accepted = traj.setComplexManeuver(maneuver);
        log_.info("Trajectory response to complex maneuver = " + accepted);
        return new TrajectoryPlanningResponse();
    }

    @Override
    public void onReceiveNegotiationRequest(String plan) {
        // TODO Wait for the DELEGATE messages from its leader and transit to LeaderState.
    }

}
