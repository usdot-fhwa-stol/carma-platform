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
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The LeaderWaitingState is a state when the platooning algorithm is enabled and is waiting for a candidate to join.
 * The host vehicle will stay in current state for ~30 seconds to wait the target vehicle to join.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to PlatoonLeaderState when either the target vehicle is joined or the target fails to join(timeout)
 * In this state, the plug-in will not insert any maneuvers into the trajectory and it will not response to any request messages.
 * In this state, the leader stops sending out heart beat message but it will keep broadcast STATUS if necessary
 */
public class LeaderWaitingState implements IPlatooningState {
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    
    // The target vehicle we are currently waiting for
    protected String               applicantId;
    protected long                 waitingStartTime;
    
    public LeaderWaitingState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator, String applicantId) {
        this.plugin               = plugin;
        this.log                  = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.applicantId          = applicantId;
        this.waitingStartTime     = System.currentTimeMillis();
        this.plugin.handleMobilityPath.set(false);
    }
    
    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // check if we have a platooning window, if not we change back to standby state
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), PlatooningPlugin.PLATOONING_FLAG)) {
            // as a leader, the actual plan job is delegated to its default cruising plug-in
            log.debug("Not insert any maneuvers in trajectory at LeaderWaitingState in " + traj.toString());
        } else {
            log.info(traj.toString() + " does not have any platooning plan window, transiting to Standby");
            // TODO Since it is currently waiting on someone else to join, it might be a good idea to send out some kind of
            // ABORT mobility message to inform another vehicle to stop accelerating and to abort its current JOIN plan 
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // In this state, it is wait on target vehicle's request message to let it actually join
        boolean isTargetVehicle = msg.getHeader().getSenderId().equals(applicantId);
        boolean isCandidateJoin = msg.getPlanType().getType() == PlanType.PLATOON_FOLLOWER_JOIN;
        if(isTargetVehicle && isCandidateJoin) {
            log.debug("Target vehicle " + applicantId + " is actually joining.");
            log.debug("Changing to PlatoonLeaderState and send ACK to target vehicle");
            plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
            return MobilityRequestResponse.ACK;
        } else {
            log.debug("Received platoon request with vehicle id = " + msg.getHeader().getSenderId());
            log.debug("The request type is " + msg.getPlanType().getType() + " and we choose to ignore");
            return MobilityRequestResponse.NO_RESPONSE;
        }
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        // We still need to handle STATUS operation message from our platoon
        String strategyParams = msg.getStrategyParams();
        boolean isPlatoonStatusMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        if(isPlatoonStatusMsg) {
            String vehicleID = msg.getHeader().getSenderId();
            String platoonId = msg.getHeader().getPlanId();
            String statusParams = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
            plugin.platoonManager.memberUpdates(vehicleID, platoonId, msg.getHeader().getSenderBsmId(), statusParams);
            log.debug("Received platoon status message from " + msg.getHeader().getSenderId());
        } else {
            log.debug("Received a mobility operation message with params " + msg.getStrategyParams() + " but ignored.");
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // We are not sending out mobility request so we do not handle any responses
    }

    @Override
    public void run() {
        try {
            // The job for this loop is:
            // 1. Check the waiting time for LeaderWaiting state, if timeouts we transit to normal leader state
            // 2. Send out operation STATUS messages
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Task 1
                if(tsStart - this.waitingStartTime > plugin.waitingStateTimeout * 1000) {
                    //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
                    log.info("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
                    plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                }
                // Task 2
                MobilityOperation status = plugin.mobilityOperationPublisher.newMessage();
                composeMobilityOperationStatus(status);
                plugin.mobilityOperationPublisher.publish(status);
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(plugin.statusMessageInterval - (tsEnd - tsStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    @Override
    public String toString() {
        return "LeaderWaitingState";
    }
    
    // This method compose mobility operation STATUS message
    private void composeMobilityOperationStatus(MobilityOperation msg) {
        msg.getHeader().setPlanId(plugin.platoonManager.currentPlatoonID);
        // This message is for broadcast
        msg.getHeader().setRecipientId("");
        // TODO need to have a easy way to get bsmId in plugin
        msg.getHeader().setSenderBsmId(pluginServiceLocator.getTrackingService().getCurrentBSMId());
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        double cmdSpeed = plugin.getLastSpeedCmd();
        double downtrackDistance = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
        double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
        String params = String.format(PlatooningPlugin.OPERATION_STATUS_PARAMS, cmdSpeed, downtrackDistance, currentSpeed);
        msg.setStrategyParams(params);
    }
    
}
