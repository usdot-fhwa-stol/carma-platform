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

import cav_msgs.MobilityIntro;
import cav_msgs.NewPlan;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

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
        RouteService rs = pluginServiceLocator_.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // Check if the next trajectory includes a platooning window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            log_.info("In standby state, find an avaliable plan window and change to leader state in " + traj.toString());
            plugin_.setState(new LeaderState(plugin_, log_, pluginServiceLocator_));
            // Request to replan with new state and give enough time for plugin state transition
            tpr.requestDelayedReplan(50);
        } else {
            log_.info("In standby state, asked to plan a trajectory without available winodw, ignoring " + traj.toString());
        }
        return tpr;
    }

    @Override
    public void onReceiveNegotiationMessage(NewPlan plan) {
        // ignore NewPlan message in the standby state
        if(plan instanceof NewPlan)
        log_.info("Ignore new plan message because the plugin is current in standby state");
    }
    
    @Override
    public String toString() {
        return "StandbyState";
    }

    @Override
    public MobilityIntro getNewOutboundIntroMessage() {
        return null;
    }

    @Override
    public void checkCurrentState() {
        // We can only transit to other state from current state when the coming trajectory has an available window, so No-Op
    }
}
