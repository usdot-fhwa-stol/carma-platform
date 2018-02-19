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
import java.util.UUID;

import cav_msgs.BasicVehicleClass;
import cav_msgs.MobilityIntro;
import cav_msgs.NewPlan;
import cav_msgs.PlanType;
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
        TrajectoryPlanningResponse response = new TrajectoryPlanningResponse();
        // Check if we have a plan window
        if(pluginServiceLocator_.getRouteService().isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin_.PLATOONING_FLAG)) {
            // Insert a PlatooningManeuver in the earliest legal window if it is in follower state
            // Extend the trajectpory end location to the very end of the same window
            double endOfCurrentRoute = pluginServiceLocator_.getRouteService().getSpeedLimitsOnRoute().last().getLocation();
            double[] platooningWindow = pluginServiceLocator_.getRouteService()
                    .getAlgorithmEnabledWindowInRange(traj.getStartLocation(), endOfCurrentRoute, plugin_.PLATOONING_FLAG);
            // Check if we have a long enough plan window
            if(platooningWindow != null && Math.abs(platooningWindow[1] - platooningWindow[0]) < plugin_.minimumManeuverLength) {
                log_.warn("Cannot find enough window to plan a platooning complex maneuver and change to standby state");
                plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
                return response;
            } else {
                log_.info("Found enough platoon window: [" + platooningWindow[0] + ", " + platooningWindow[1] + "]");
                if(traj.getEndLocation() < platooningWindow[1]) {
                    response.requestLongerTrajectory(platooningWindow[1]);
                    log_.info("Requesting a longer trajectory");
                    return response;
                }
            }
            List<IManeuver> maneuvers = traj.getManeuvers();
            // Check if the trajectory is empty
            if(!maneuvers.isEmpty()) {
                response.requestHigherPriority();
                return response;
            }
            
            PlatooningManeuver maneuver = new PlatooningManeuver(
                    plugin_,
                    plugin_.commandGenerator,
                    pluginServiceLocator_.getManeuverPlanner().getManeuverInputs(),
                    pluginServiceLocator_.getManeuverPlanner().getGuidanceCommands(),
                    AccStrategyManager.newAccStrategy(),
                    platooningWindow[0], platooningWindow[1],
                    1.0, 100.0); // TODO the last two are dummy variables, replace them later if possible
            boolean accepted = traj.setComplexManeuver(maneuver);
            log_.info("Planned complex maneuver: " + maneuver.toString());
            log_.info("Trajectory response to complex maneuver = " + accepted);
            return new TrajectoryPlanningResponse();
        } else {
            // Put plugin in StandbyState when platooning algorithm in disabled in the next trajectory
            // TODO it may need to send out some mobility messages when the transition happened
            plugin_.setState(new StandbyState(plugin_, log_, pluginServiceLocator_));
            return new TrajectoryPlanningResponse();
        }
    }

    @Override
    public void onReceiveNegotiationMessage(NewPlan plan) {
        // update the info of platoon members in front of us
        // TODO Should use another type here in future, for now it is just the general type for platooning plan message  
        if(plan.getType().getType() == PlanType.SEARCHING_FOR_PLATOON) {
            plugin_.platoonManager.memberUpdates(plan);
        }
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
        // Change to leader state if the subject vehicle became the first one in the platoon and re-plan the trajectory
        if(plugin_.platoonManager.platoon.size() == 0) {
            plugin_.setState(new LeaderState(plugin_, log_, pluginServiceLocator_));
            pluginServiceLocator_.getArbitratorService().notifyTrajectoryFailure();
        }
    }
}
