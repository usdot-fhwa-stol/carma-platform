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
 * The LeaderState is a state when the platooning algorithm is enabled and the host vehicle is acting as the leader for zero or many vehicles
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory
 * It will transit to FollowerState when it found an available leader in front of it in the same lane
 * In this state, the plug-in will not insert any maneuvers into the trajectory,
 * but it will try to join another platoon or to let others join its platoon by
 * sending out introduction message with platooning status and also handle mobility messages from other vehicles
 */
public class LeaderWaitingState implements IPlatooningState {
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    // The target vehicle we are currently waiting for
    protected String               targetVehicleId;
    
    public LeaderWaitingState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator, String targetId) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.targetVehicleId = targetId;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // check if we have a platooning window, if not we change back to standby state
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            // as a leader, the actual plan job is delegated to its default cruising plug-in
            log.debug("Not insert any maneuvers in trajectory at LeaderWaitingState in " + traj.toString());
        } else {
            log.info(traj.toString() + " does not have any platooning plan window, transiting to Standby");
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        
    }
    
    @Override
    public String toString() {
        return "LeaderWaitingState";
    }
    
//    @Override
//    public MobilityIntro getNewOutboundIntroMessage() {
//        MobilityIntro message = plugin_.mobilityIntroPublisher.newMessage(); 
//        message.getHeader().setSenderId(plugin_.STATIC_ID);
//        message.getHeader().setRecipientId("00000000-0000-0000-0000-000000000000");
//        message.getHeader().setPlanId(UUID.randomUUID().toString());
//        message.getHeader().setTimestamp(System.currentTimeMillis());
//        message.getMyEntityType().setType(BasicVehicleClass.DEFAULT_PASSENGER_VEHICLE);
//        double speed = plugin_.maneuverInputs.getCurrentSpeed();
//        message.setForwardSpeed((float) speed);
//        message.setMyLaneId((byte) plugin_.maneuverInputs.getCurrentLane());
//        // TODO change it to real road name, just a placeholder for now
//        message.setMyRoadwayLink("[Test track]");
//        double dowtrack = plugin_.maneuverInputs.getDistanceFromRouteStart();
//        message.setMyRoadwayLinkPosition((short) dowtrack);
//        message.setExpiration(System.currentTimeMillis() + 5000); //TODO not in use for now
//        // TODO need to accommodate new mobility message format
//        message.getPlanType().setType(PlanType.UNKNOWN);
//        double cmdSpeed = plugin_.cmdSpeedSub.getLastMessage() != null ? plugin_.cmdSpeedSub.getLastMessage().getSpeed() : 0.0;  
//        String cap = String.format("[CMDSPEED:%.1f, DOWNTRACK:%.1f, SPEED:%.1f]", cmdSpeed, dowtrack, speed);
//        message.setCapabilities(cap);
//        log_.info("V2V", "Platooning leader has built a mobility message with planId = " + message.getHeader().getPlanId());
//        log_.info("V2V", message.getHeader().getPlanId() + " has capabilities string: " + cap);
//        return message;
//    }
    
}
