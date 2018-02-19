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

import java.util.UUID;

import cav_msgs.BasicVehicleClass;
import cav_msgs.MobilityIntro;
import cav_msgs.NewPlan;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The LeaderState is a state when the platooning algorithm is enabled and the host vehicle is acting as the leader for zero or many vehicles
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory
 * It will transit to FollowerState when it found an available leader in front of it in the same lane
 * In this state, the pulgin will not insert any maneuvers into the trajectory,
 * but it will try to join another platoon or to let others join its platoon by
 * sending out introduction message with platooning status and also handle mobility messages from other vehicles
 */
public class LeaderState implements IPlatooningState {
    
    protected PlatooningPlugin plugin_;
    protected ILogger log_;
    protected PluginServiceLocator pluginServiceLocator_;
    protected long stateTransitionTime = Long.MAX_VALUE;
    
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
            // As the leader, no need to plan any Platooning maneuvers
            log_.info("Not insert any maneuvers in trajectory at leader state " + traj.toString());
        } else {
            // Put plugin in StandbyState when platooning algorithm in disabled in the next trajectory
            // Need some time delay to let other vehicles finish current complex maneuver before we
            // stop sending mobility messages and transit to the new state
            // We assume the subjust vehicle will be at speed limit at the start for the next trajectory
            double currentDistance = pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance();
            double currentSpeed = pluginServiceLocator_.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
            double speedAtTrajectoryStart = pluginServiceLocator_.getRouteService().getSpeedLimitAtLocation(traj.getStartLocation()).getLimit();
            double speedAvg = (currentSpeed + speedAtTrajectoryStart) / 2;
            int timeDelay = (int) ((traj.getStartLocation() - currentDistance) / speedAvg);
            stateTransitionTime = System.currentTimeMillis() + timeDelay;
        }
        return tpr;
    }

    @Override
    public void onReceiveNegotiationMessage(NewPlan plan) {
        // Only care about SEARCHING_FOR_PLATOON message for now
        if(plan.getType().getType() == PlanType.SEARCHING_FOR_PLATOON) {
            plugin_.platoonManager.memberUpdates(plan);
            log_.info("Receiving new platooning plan message " + plan.getPlanId() + " from " + plan.getSenderId());
        }
    }
    
    @Override
    public String toString() {
        return "LeaderState";
    }

    @Override
    public MobilityIntro getNewOutboundIntroMessage() {
        MobilityIntro message = plugin_.mobilityIntroPublisher.newMessage(); 
        message.getHeader().setSenderId(plugin_.STATIC_ID);
        message.getHeader().setRecipientId("00000000-0000-0000-0000-000000000000");
        message.getHeader().setPlanId(UUID.randomUUID().toString());
        message.getHeader().setTimestamp(System.currentTimeMillis());
        message.getMyEntityType().setType(BasicVehicleClass.DEFAULT_PASSENGER_VEHICLE);
        double speed = plugin_.maneuverInputs.getCurrentSpeed();
        message.setForwardSpeed((float) speed);
        message.setMyLaneId((byte) plugin_.maneuverInputs.getCurrentLane());
        // TODO change it to real road name, just a placeholder for now
        message.setMyRoadwayLink("[Test track]");
        double dowtrack = plugin_.maneuverInputs.getDistanceFromRouteStart();
        message.setMyRoadwayLinkPosition((short) dowtrack);
        message.setExpiration(System.currentTimeMillis() + 5000); //TODO not in use for now
        message.getPlanType().setType(PlanType.SEARCHING_FOR_PLATOON);
        double cmdSpeed = plugin_.cmdSpeedSub.getLastMessage() != null ? plugin_.cmdSpeedSub.getLastMessage().getSpeed() : 0.0;  
        String cap = String.format("[CMDSPEED:%.1f, DOWNTRACK:%.1f, SPEED:%.1f]", cmdSpeed, dowtrack, speed);
        message.setCapabilities(cap);
        log_.info("V2V", "Platooning leader has built a mobility message with planId = " + message.getHeader().getPlanId());
        log_.info("V2V", message.getHeader().getPlanId() + " has capabilities string: " + cap);
        return message;
    }

    @Override
    public void checkCurrentState() {
        // If we found we platooning vehicle in front of us, we change to follower state
        if(plugin_.platoonManager.platoon.size() != 0) {
            plugin_.setState(new FollowerState(plugin_, log_, pluginServiceLocator_));
            pluginServiceLocator_.getArbitratorService().notifyTrajectoryFailure();
        }
        // Transit to standby state when the current trajectory is finished
        if(System.currentTimeMillis() > stateTransitionTime) {
            plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
            stateTransitionTime = Long.MAX_VALUE;
        }
    }
    
}
