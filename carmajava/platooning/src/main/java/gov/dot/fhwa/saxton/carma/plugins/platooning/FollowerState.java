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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.List;

import cav_msgs.MobilityAck;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * The FollowerState is a state when the platooning algorithm is enabled and the host vehicle is not the leader.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to LeaderState when either it cannot maintain the gap with the front vehicle
 * or its leader sends a negotiation message to assign it as the new leader.
 * In this state, the plugin will insert a PlatooningManeuver into the trajectory and control the vehicle.
 */
public class FollowerState implements IPlatooningState {

    protected static final long DEFAULT_LOOP_SLEEP_MS = 1000;
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
        // Check if we have a plan window
        if(pluginServiceLocator_.getRouteService().isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            // Insert a PlatooningManeuver in the earliest legal window if it is in follower state
            double complexManeuverStartLocation = traj.getStartLocation();
            List<IManeuver> maneuvers = traj.getManeuvers();
            if(!maneuvers.isEmpty()) {
                complexManeuverStartLocation = maneuvers.get(maneuvers.size() - 1).getEndDistance(); 
            }
            double[] window = pluginServiceLocator_.getRouteService().getAlgorithmEnabledWindowInRange(complexManeuverStartLocation, traj.getEndLocation(), plugin_.PLATOONING_FLAG);
            // Check if we have a long enough plan window
            if(window != null && Math.abs(window[1] - window[0]) < plugin_.minimumManeuverLength) {
                log_.warn("Cannot find a legal window to plan a platooning complex maneuver");
                // TODO it may need to use arbitrator service to replan or request a longer trajectory
                TrajectoryPlanningResponse response = new TrajectoryPlanningResponse();
                response.requestLongerTrajectory(traj.getEndLocation() + (plugin_.minimumManeuverLength - Math.abs(window[1] - window[0])));
                return response;
            }
            // Check if the plan window is at the start of the trajectory
            // TODO make isFloatingPointEqual a method
            if(Math.abs(traj.getStartLocation() - window[0]) < 0.1) {
                // TODO send leave message before changing to leader state
                plugin_.setState(new LeaderState(plugin_, log_, pluginServiceLocator_));
                return new TrajectoryPlanningResponse();
            }
            PlatooningManeuver maneuver = new PlatooningManeuver(
                    plugin_,
                    plugin_.commandGenerator,
                    pluginServiceLocator_.getManeuverPlanner().getManeuverInputs(),
                    pluginServiceLocator_.getManeuverPlanner().getGuidanceCommands(),
                    AccStrategyManager.newAccStrategy(),
                    window[0], window[1],
                    1.0, 100.0); // TODO the last two are dummy variables, replace them later if possible
            boolean accepted = traj.setComplexManeuver(maneuver);
            log_.info("Trajectory response to complex maneuver = " + accepted + " in the window [" + window[0] + ", " + window[0] + "]");
            return new TrajectoryPlanningResponse();
        } else {
            // Put plugin in StandbyState when platooning algorithm in disabled in the next trajectory
            // TODO it may need to send out some mobility messages when the transition happened
            // Maybe need some time delay to finish current complex maneuver before the transition happened
            plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
            return new TrajectoryPlanningResponse();
        }
    }

    @Override
    public boolean onReceiveNegotiationRequest(String plan) {
        // TODO Wait for the DELEGATE messages from its leader and transit to LeaderState.
        // TODO Wait for the UPDATE messages from the members in this platoon
        return false;
    }
    
    @Override
    public void onReceivePlanResponse(MobilityAck ack) {
        // This state is not sending plans out so no action required 
    }
}
