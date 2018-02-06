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

import cav_msgs.MobilityAck;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The LeaderState is a state when the platooning algorithm is enabled and the host vehicle is acting as the leader for zero or many vehicles.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to FollowerState when it decide to join another platoon
 * after receiving positive ACK mobility message from another leader for a JOIN mobility message.
 * In this state, the pulgin will not insert any maneuvers into the trajectory,
 * but it will try to join another platoon by negotiation and handle handle all vehicles who try to join its platoon.
 */
public class LeaderState implements IPlatooningState {
    
    protected static final long DEFAULT_LOOP_SLEEP_MS = 1000;
    protected PlatooningPlugin plugin_;
    protected ILogger log_;
    protected PluginServiceLocator pluginServiceLocator_;
    
    public LeaderState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        plugin_ = plugin;
        log_ = log;
        pluginServiceLocator_ = pluginServiceLocator;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator_.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            //TODO maybe we need to check the length of the window and require a longer trajectory
            return tpr;
        } else {
            // Put plugin in StandbyState when platooning algorithm in disabled in the next trajectory
            // TODO it need to send out some mobility messages to delegate its job
            plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
        }
        return tpr;
    }

    @Override
    public boolean onReceiveNegotiationRequest(String plan) {
        //handle Join and Leave messages and update with platoon manager accordingly
        return false;
    }
    
    @Override
    public void loop() throws InterruptedException {
        // constantly send Join message if this leader has no member
        // constantly send Update message if this leader has members
        try {
            Thread.sleep(DEFAULT_LOOP_SLEEP_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw e;
        }
    }
    
    @Override
    public void onReceivePlanResponse(MobilityAck ack) {
        // If we received a positive response for our Join message
        // we change to Follower state and notify arbitrator to replan
    }
    
    @Override
    public String toString() {
        return "LeaderState";
    }
    
}
