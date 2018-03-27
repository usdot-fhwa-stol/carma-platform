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

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to SingleVehiclePlatoonState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the plug-in will not insert any maneuvers into a trajectory and will ignore all negotiation messages.
 */
public class StandbyState implements IPlatooningState {
    
    protected static int LOOP_SLEEP_TIME   = 1000;
    protected static int REPLAN_DELAY_TIME = 50;
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    
    public StandbyState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin               = plugin;
        this.log                  = log;
        this.pluginServiceLocator = pluginServiceLocator;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // Check if the next trajectory includes a platooning window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            log.debug("In standby state, find an avaliable plan window in " + traj.toString());
            plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
            // Request to re-plan with new leader state
            log.debug("Change to leader state and request to re-plan");
            tpr.requestDelayedReplan(REPLAN_DELAY_TIME);
        } else {
            log.debug("In standby state, asked to plan a trajectory without available winodw, ignoring...");
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // In standby state, the plugin has no responsible for replying to any request messages
        log.info("CACC plugin receives a mobility request but chooses to ignore. PlanId = " + msg.getHeader().getPlanId());
        return MobilityRequestResponse.NO_RESPONSE;
    }
    
    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // In standby state, it will not send out any requests so it will also ignore all responses
        log.info("CACC plugin receives a mobility response but chooses to ignore. PlanId = " + msg.getHeader().getPlanId());
    }
    
    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        // In standby state, it will ignore operation message since it is not actively operating
        log.info("CACC plugin receives a mobility operation but chooses to ignore. PlanId = " + msg.getHeader().getPlanId());
    }
    
    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                Thread.sleep(LOOP_SLEEP_TIME);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    @Override
    public String toString() {
        return "StandbyState";
    }

}
