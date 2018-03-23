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
public class LeaderState implements IPlatooningState {
    
    enum LeaderSubstate {
        // the regular leader state which means the host vehicle is leading a non-zero length platoon  
        PLATOON_LEADER,
        // the single vehicle state which is acting as a leader but it does not actually lead any vehicles
        SINGLE_VEHICLE,
        // the leader has accepted a join request and is waiting on a specific vehicle to join
        LEADER_WAITING,
        // the leader is trying to join another platoon in front of him by being at the rear of that platoon
        CANDIDATE_FOLLOWER
    }
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    protected LeaderSubstate       substate = LeaderSubstate.SINGLE_VEHICLE; 
    
    public LeaderState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin               = plugin;
        this.log                  = log;
        this.pluginServiceLocator = pluginServiceLocator;
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        switch(this.substate) {
            case SINGLE_VEHICLE:
            case LEADER_WAITING:
            case PLATOON_LEADER:
                if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
                    // as the leader, the actual plan job is delegated to its default cruising plug-in
                    log.info("Not insert any maneuvers in trajectory at leader state in " + traj.toString());
                } else {
                    // if the next trajectory did not have a plan window, we transit to standby state
                    log.info(traj.toString() + " does not have any platooning plan window, transiting to Standby");
                    plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
                }
                break;
            case CANDIDATE_FOLLOWER:
                // TODO insert a speed-up maneuver and a steady speed maneuver to stay at the rear of the target platoon 
                break;
        }
        return tpr;
    }
    
    @Override
    public String toString() {
        return "LeaderState";
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        
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
