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
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.NewPlan;
import cav_msgs.PlanType;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlanStatus;

/**
 * The FollowerState is a state when the platooning algorithm is enabled and the host vehicle is not the leader.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to LeaderState when either it cannot maintain the gap with the front vehicle
 * or its leader sends a negotiation message to assign it as the new leader.
 * In this state, the plugin will insert a PlatooningManeuver into the trajectory and control the vehicle.
 */
public class FollowerState implements IPlatooningState {

    protected PlatooningPlugin plugin;
    protected ILogger log;
    protected PluginServiceLocator pluginServiceLocator;
    
    public FollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // Check if we have a plan window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            // Insert a PlatooningManeuver in the earliest legal window if it is in follower state
            double[] platooningWindow = rs.getAlgorithmEnabledWindowInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG);
            // Check if we have a long enough plan window
            if(platooningWindow != null && Math.abs(platooningWindow[1] - platooningWindow[0]) < plugin.getMinimumManeuverLength()) {
                log.warn("Cannot find enough window to plan a platooning complex maneuver and change to standby state");
                plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
            } else {
                // plan a complex maneuver in the current platoon window
                log.debug("Found enough platoon window: [" + platooningWindow[0] + ", " + platooningWindow[1] + "] and start to plan CM");
                PlatooningManeuver maneuver = new PlatooningManeuver(
                        plugin, plugin.getCommandGenerator(),
                        pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
                        pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(),
                        AccStrategyManager.newAccStrategy(),
                        platooningWindow[0], platooningWindow[1],
                        1.0, 100.0); // TODO the last two are dummy variables, replace them later if possible
                boolean isAccepted = traj.setComplexManeuver(maneuver);
                if(isAccepted) {
                    log.debug("Planned and inserted a complex maneuver: " + maneuver.toString());
                } else {
                    log.debug("Planned a complex maneuver: " + maneuver.toString() + " but it cannot be inserted into trajectory");
                    log.debug("Request a higher priority in plan");
                    tpr.requestHigherPriority();
                }
            }
        } else {
            // Put plugin in StandbyState when platooning algorithm in disabled in the next trajectory
            log.debug("Plugin is disabled on the next trajectory. Change to StandbyState");
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // No need to response as a follower
        return MobilityRequestResponse.NO_RESPONSE;
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        String strategyParams = msg.getStrategyParams();
        // In the current state, we care about the STATUS message
        boolean isPlatoonStatusMsg = strategyParams.startsWith("STATUS");
        // if it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:5.0
        if(isPlatoonStatusMsg) {
            // we only care about platoon status message when the strategy id matches    
            if(plugin.getPlatoonManager().getCurrentPlatoonID().equals(msg.getStrategyId())) {
                String vehicleID = msg.getHeader().getSenderId();
                String statusParams    = strategyParams.split("|")[1];
                log.info("Receive operation message from our platoon from vehicle: " + vehicleID);
                plugin.getPlatoonManager().memberUpdates(vehicleID, statusParams);
            } else {
                log.debug("Ignore other STATUS messages from other platoon");
            }
        } else {
            log.debug("Ignore other non-STATUS messages with params " + strategyParams);
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // No need to handle response since we are not sending any requests
    }
    
    @Override
    public void run() {
        // This is an interrupt-safe loop 
        // This loop does two things:
        // 1. Publish operation status every 100 milliseconds
        // 2. Change to leader state if there is no active leader
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Job 1
                MobilityOperation status = plugin.getMobilityOperationPublisher().newMessage();
                composeMobilityOperationStatus(status);
                plugin.getMobilityOperationPublisher().publish(status);
                // Job 2
                // TODO get the number of vehicles in this platoon who is in front of us
                if(plugin.getPlatoonManager().getPlatooningSize() == 0) {
                    plugin.setState(new PlatoonLeaderState(plugin, log, pluginServiceLocator));
                    // We need to change to the leader state and which requests a re-plan
                    pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
                }
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(plugin.getOperationUpdatesIntervalLength() - (tsEnd - tsStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    @Override
    public String toString() {
        return "FollowerState";
    }
    
    // This method compose mobility operation STATUS message
    private void composeMobilityOperationStatus(MobilityOperation msg) {
        msg.getHeader().setPlanId("");
        // This message is for broadcast
        msg.getHeader().setRecipientId("");
        // TODO need to have a easy way to get bsmId in plugin
        msg.getHeader().setSenderBsmId("FFFFFFFF");
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategyId(plugin.getPlatoonManager().getCurrentPlatoonID());
        msg.setStrategy(plugin.MOBILITY_STRATEGY);
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        SpeedAccel lastCmdSpeed = plugin.getCmdSpeedSub().getLastMessage();
        double cmdSpeed = lastCmdSpeed == null ? 0.0 : lastCmdSpeed.getSpeed();
        double downtrackDistance = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
        double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
        String params = String.format("STATUS|CMDSPEED:%.2f,DOWNTRACK:%.2f,SPEED:%.2f", cmdSpeed, downtrackDistance, currentSpeed);
        msg.setStrategyParams(params);
    }
}
