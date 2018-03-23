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

import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.Message;

import cav_msgs.MobilityRequest;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The StandbyState is a state when the platooning algorithm is current disabled on the route.
 * It will transit to LeaderState when it knows the algorithm will be enabled in the next trajectory.
 * In this state, the plug-in will not insert any maneuvers into a trajectory and ignore all negotiation messages.
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
            tpr.requestDelayedReplan(100);
        } else {
            log_.info("In standby state, asked to plan a trajectory without available winodw, ignoring " + traj.toString());
        }
        return tpr;
    }

    @Override
    public void onReceiveMobilityMessgae(Message mobilityMessage) {
        if(mobilityMessage instanceof MobilityRequest) {
            // TODO send NACK message in the standby state for mobility request which targets the host vehicle
            log_.info("Send negative response for targeted MobilityRequest because we are in standby state.");
        } else {
            log_.info("Ignore other mobility messages because the plugin is current in standby state");
        }
        
    }
    
    @Override
    public List<Message> getNewMobilityOutbound() {
        // return an empty list at standby state
        // TODO depends on the routing system, we may need to add mobility nack in this list 
        return new ArrayList<Message>();
    }
    
    @Override
    public void loop() throws InterruptedException {
        Thread.sleep(1000);
    }
    
    @Override
    public String toString() {
        return "StandbyState";
    }
}
