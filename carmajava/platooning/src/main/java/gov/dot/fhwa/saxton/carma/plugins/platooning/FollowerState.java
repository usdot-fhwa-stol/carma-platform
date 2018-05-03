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

import java.util.Arrays;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.IndicatorStatus;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The FollowerState is a state when the platooning algorithm is enabled and the host vehicle is a follower.
 * It will transit to StandbyState when the algorithm is disabled in the next trajectory.
 * It will transit to PlatoonLeaderState when it found there is no vehicle in the same platoon in front of it.
 * In this state, the plugin will insert a PlatooningManeuver into the trajectory and control the vehicle.
 * It will also keep sending and handling mobility operation STATUS messages.
 */
public class FollowerState implements IPlatooningState {

    protected static int LEADER_TIMEOUT_COUNTER_LIMIT = 5;
    
    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   int                  noLeaderUpdatesCounter = 0;
    
    
    public FollowerState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.plugin.handleMobilityPath.set(false);
        updateLightBar();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // Check if we have a plan window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), PlatooningPlugin.PLATOONING_FLAG)) {
            // Insert a PlatooningManeuver in the earliest legal window
            double[] platooningWindow = rs.getAlgorithmEnabledWindowInRange(traj.getStartLocation(), traj.getEndLocation(), PlatooningPlugin.PLATOONING_FLAG);
            // Check if we have a long enough plan window
            if(platooningWindow != null && Math.abs(platooningWindow[1] - platooningWindow[0]) < plugin.minimumManeuverLength) {
                log.warn("Cannot find a long enough window to plan a platooning complex maneuver in " + traj.toString());
                log.debug("Change back to Standby state");
                plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
            } else {
                // Plan a complex maneuver in the current platoon window
                log.debug("Found a long enough platoon window: " + Arrays.toString(platooningWindow) + " and start to plan a complex maneuver");
                // TODO the last two are dummy variables, replace them later if possible
                PlatooningManeuver maneuver = new PlatooningManeuver(
                        plugin, plugin.commandGenerator,
                        pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
                        pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(),
                        AccStrategyManager.newAccStrategy(),
                        platooningWindow[0], platooningWindow[1], 1.0, 100.0);
                boolean isAccepted = traj.setComplexManeuver(maneuver);
                if(isAccepted) {
                    log.debug("Planned and inserted a complex maneuver: " + maneuver.toString());
                } else {
                    log.debug("Planned a complex maneuver: " + maneuver.toString() + " but it cannot be inserted into trajectory");
                    log.debug("Assuming that space is already planned and request a higher priority in plan");
                    tpr.requestHigherPriority();
                }
            }
        } else {
            // Put plugin in StandbyState when platooning algorithm is disabled in the next trajectory
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
        boolean isPlatoonStatusMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        boolean isPlatoonInfoMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_INFO_TYPE);
        // If it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
        if(isPlatoonStatusMsg) {
            String vehicleID = msg.getHeader().getSenderId();
            String platoonID = msg.getHeader().getPlanId();
            String statusParams = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
            log.debug("Receive operation message from vehicle: " + vehicleID);
            plugin.platoonManager.memberUpdates(vehicleID, platoonID, msg.getHeader().getSenderBsmId(), statusParams);
        } else if(isPlatoonInfoMsg) {
            if(msg.getHeader().getSenderId().equals(plugin.platoonManager.leaderID)) {
                String infoParams = strategyParams.substring(PlatooningPlugin.OPERATION_INFO_TYPE.length() + 1);
                plugin.platoonManager.platoonSize = Integer.parseInt(infoParams.split(",")[3].split(":")[1]);
                log.debug("Update from the lead: the current platoon size is " + plugin.platoonManager.platoonSize);
            }
        } else {
            log.debug("Ignore other operation messages with params: " + strategyParams);
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // No need to handle response since we are not sending any requests
    }
    
    @Override
    public void run() {
        // This is an interrupted-safe loop 
        // This loop has two tasks:
        // 1. Publish operation status every 100 milliseconds
        // 2. Change to leader state if there is no active leader
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Job 1
                MobilityOperation status = plugin.mobilityOperationPublisher.newMessage();
                composeMobilityOperationStatus(status);
                plugin.mobilityOperationPublisher.publish(status);
                // Job 2
                // Get the number of vehicles in this platoon who is in front of us
                int vehicleInFront = plugin.platoonManager.getNumberOfVehicleInFront(); 
                if(vehicleInFront == 0) {
                    noLeaderUpdatesCounter++;
                    if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
                        log.debug("noLeaderUpdatesCounter = " + noLeaderUpdatesCounter + " and change to leader state");
                        plugin.platoonManager.changeFromFollowerToLeader();
                        plugin.setState(new LeaderState(plugin, log, pluginServiceLocator));
                        // Because we need to abort current complex maneuver, we call arbitrator to re-plan
                        pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
                    }
                } else {
                    // reset counter to zero when we get updates again
                    noLeaderUpdatesCounter = 0;
                }
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(PlatooningPlugin.STATUS_INTERVAL_LENGTH - (tsEnd - tsStart), 0);
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
        msg.getHeader().setPlanId(plugin.platoonManager.currentPlatoonID);
        // All platoon mobility operation message is just for broadcast
        msg.getHeader().setRecipientId("");
        // TODO Need to have a easy way to get bsmId from plugin
        msg.getHeader().setSenderBsmId("FFFFFFFF");
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategy(PlatooningPlugin.MOBILITY_STRATEGY);
        double cmdSpeed = plugin.getLastSpeedCmd();
        // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        String statusParams = String.format(PlatooningPlugin.OPERATION_STATUS_PARAMS,
                                            cmdSpeed, pluginServiceLocator.getRouteService().getCurrentDowntrackDistance(),
                                            pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
        msg.setStrategyParams(statusParams);
        log.debug("Composed a mobility operation message with params " + msg.getStrategyParams());
    }

    /**
     * Helper function to update the light bar
     */
    private void updateLightBar() {
        plugin.setLightBarStatus(IndicatorStatus.FLASH);
    }
    
}
