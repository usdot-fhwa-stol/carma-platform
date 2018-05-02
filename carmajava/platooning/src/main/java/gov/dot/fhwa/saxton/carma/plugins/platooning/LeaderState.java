package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlanType;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.IndicatorStatus;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;

/**
 * The PlatoonLeaderState is a state which platooning algorithm is enabled on the current trajectory
 * or the next trajectory no matter whether it actually lead any vehicles or not. This state will transit
 * to Standby state when the algorithm is disabled on the next trajectory; it will transit to CandidateFollower
 * state if it is trying to join another platoon in front of it; it can also transit to LeaderWaiting state
 * if it is waiting on another vehicle behind it to join from its platoon rear. In this state, it will not
 * actively plan any maneuvers but it will handle any JOIN request, actively look for any available platoons
 * in front of it and keep broadcasting heart-beat mobility operation INFO message to inform its existence.
 */
public class LeaderState implements IPlatooningState {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   PlatoonPlan          currentPlan;
    private   long                 lastHeartBeatTime     = 0;
    private   String               potentialNewPlatoonId = "";

    public LeaderState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
        this.plugin.handleMobilityPath.set(false);
        // Update the light bar
        updateLightBar();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is a platooning window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), PlatooningPlugin.PLATOONING_FLAG)) {
            // As a leader, the actual plan job is done by the default cruising plug-in
            log.debug("Not insert any maneuvers in trajectory at PlatoonLeaderState in " + traj.toString());
        } else {
            log.info("There is no platoon window in " + traj.toString() + ". Change back to Standby state.");
            // TODO Send out mobility request message to inform its LEAVE and delegate the leader job properly
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // If we received a JOIN request, we decide whether we allow that CAV to join.
        // If we agree on its join, we need to change to LeaderWaiting state.
        if(msg.getPlanType().getType() == (PlanType.JOIN_PLATOON_AT_REAR)) {
            // We are currently checking two basic JOIN conditions:
            //     1. The size limitation on current platoon based on the plugin's parameters.
            //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
            // TODO We ignore the lane information for now and assume the applicant is in the same lane with us.
            MobilityHeader msgHeader = msg.getHeader();
            String params = msg.getStrategyParams();
            String applicantId = msgHeader.getSenderId();
            log.debug("Receive mobility JOIN request from " + applicantId + " and PlanId = " + msgHeader.getPlanId());
            log.debug("The strategy parameters are " + params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,SPEED:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            int applicantSize = Integer.parseInt(params.split(",")[0].split(":")[1]);
            double applicantCurrentSpeed = Double.parseDouble(params.split(",")[1].split(":")[1]);
            double applicantCurrentDtd = Double.parseDouble(params.split(",")[2].split(":")[1]);
            // Check if we have enough room for that applicant
            int currentPlatoonSize = plugin.platoonManager.getPlatooningSize();
            boolean hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= plugin.maxPlatoonSize;
            if(hasEnoughRoomInPlatoon) {
                log.debug("The current platoon has enough room for the applicant with size " + applicantSize);
                double currentRearDtd = plugin.platoonManager.getPlatoonRearDowntrackDistance();
                log.debug("The current platoon rear dtd is " + currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - plugin.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                log.debug("The gap between current platoon rear and applicant is " + currentGap + "m or " + currentTimeGap + "s");
                if(currentGap < 0) {
                    log.warn("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse.NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                boolean isDistanceCloseEnough = (currentGap <= plugin.maxAllowedJoinGap) || (currentTimeGap <= plugin.maxAllowedJoinTimeGap);
                if(isDistanceCloseEnough) {
                    log.debug("The applicant is close enough and we will allow it to try to join");
                    log.debug("Change to LeaderWaitingState and waiting for " + msg.getHeader().getSenderId() + " to join");
                    plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse.ACK;
                } else {
                    log.debug("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse.NACK;
                }
            } else {
                log.debug("The current platoon does not have enough room for applicant of size " + applicantSize + ". NACK");
                return MobilityRequestResponse.NACK;
            }
        } else {
            log.debug("Received mobility request with type " + msg.getPlanType().getType() + " and ignored.");
            return MobilityRequestResponse.NO_RESPONSE;
        }
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        String strategyParams = msg.getStrategyParams();
        String senderId = msg.getHeader().getSenderId();
        String platoonId = msg.getHeader().getPlanId();
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in our current platoon
        boolean isPlatoonInfoMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_INFO_TYPE);
        boolean isPlatoonStatusMsg = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        boolean isNotInNegotiation = (this.currentPlan == null);
        if(isPlatoonInfoMsg && isNotInNegotiation) {
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d
            // TODO In future, we should remove downtrack distance from this string and send XYZ location in ECEF
            String rearVehicleBsmId = strategyParams.split(",")[0].split(":")[1];
            // We are trying to validate is the platoon rear is right in front of the host vehicle
            if(isVehicleRightInFront(rearVehicleBsmId)) {
                log.debug("Found a platoon with id = " + msg.getHeader().getPlanId() + " in front of us.");
                MobilityRequest request = plugin.mobilityRequestPublisher.newMessage();
                MobilityMessageWorker.populateInitialJoinRequest(request);
                this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), request.getHeader().getPlanId(), senderId);
                plugin.mobilityRequestPublisher.publish(request);
                log.debug("Publishing request to leader " + senderId + " with params " + request.getStrategyParams() + " and plan id = " + request.getHeader().getPlanId());
                this.potentialNewPlatoonId = platoonId;
            } else {
                log.debug("Ignore platoon with platoon id: " + msg.getHeader().getPlanId() + "because validation failure");
            } 
        } else if(isPlatoonStatusMsg) {
            // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
            if(plugin.platoonManager.currentPlatoonID.equals(platoonId)) {
                String statusParams = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
                log.debug("Receive operation status message from vehicle: " + senderId + " with params: " + statusParams);
                plugin.platoonManager.memberUpdates(senderId, msg.getHeader().getPlanId(), msg.getHeader().getSenderBsmId(), statusParams);
            } else {
                log.debug("Ignore operation status message from vehicle: " + senderId + " because it is not in our platoon");
            }
        } else {
            log.debug("Receive operation message but ignore it because isPlatoonInfoMsg = " + isPlatoonInfoMsg 
                    + ", isNotInNegotiation = " + isNotInNegotiation + " and isPlatoonStatusMsg = " + isPlatoonStatusMsg);
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // Only care the response message for the plan for which we are waiting
        if(this.currentPlan != null) {
            synchronized(this.currentPlan) {
                if(this.currentPlan != null) {
                    if(this.currentPlan.planId.equals(msg.getHeader().getPlanId()) && this.currentPlan.peerId.equals(msg.getHeader().getSenderId())) {
                        if(msg.getIsAccepted()) {
                            log.debug("Received positive response for plan id = " + this.currentPlan.planId);
                            log.debug("Change to CandidateFollower state and notify trajectory failure in order to replan");
                            // Change to candidate follower state and request a new plan to catch up with the front platoon
                            plugin.setState(new CandidateFollowerState(plugin, log, pluginServiceLocator, currentPlan.peerId, potentialNewPlatoonId));
                            pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
                        } else {
                            log.debug("Received negative response for plan id = " + this.currentPlan.planId);
                            // Forget about the previous plan totally
                            this.currentPlan = null;
                        }
                    } else {
                        log.debug("Ignore the response message because planID match: " + this.currentPlan.planId.equals(msg.getHeader().getPlanId()));
                        log.debug("My plan id = " + this.currentPlan.planId + " and response plan Id = " + msg.getHeader().getPlanId());
                        log.debug("And peer id match " + this.currentPlan.peerId.equals(msg.getHeader().getSenderId()));
                        log.debug("Expected peer id = " + this.currentPlan.peerId + " and response sender Id = " + msg.getHeader().getSenderId());
                    }
                }
            }
        } else {
            log.debug("Ignore imcoming plan because our current plan is null. plan id = " + msg.getHeader().getPlanId());
        }
    }

    @Override
    public void run() {
        // This is a loop which is safe to interrupt
        // This loop does four tasks:
        // 1. Send out heart beat mobility operation INFO message every ~3 seconds if the platoon is not full
        // 2. Updates the light bar status every ~3 seconds 
        // 3. Remove current plan if we wait for a long enough time
        // 4. Publish operation status every 100 milliseconds if we have follower
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Task 1
                boolean isTimeForHeartBeat = tsStart - lastHeartBeatTime >= PlatooningPlugin.INFO_INTERVAL_LENGTH;
                if(isTimeForHeartBeat) {
                    MobilityOperation infoOperation = plugin.mobilityOperationPublisher.newMessage();
                    composeMobilityOperation(infoOperation, "INFO");
                    plugin.mobilityOperationPublisher.publish(infoOperation);
                    lastHeartBeatTime = System.currentTimeMillis();
                    log.debug("Published heart beat platoon INFO mobility operatrion message");
                }
                // Task 2
                if (isTimeForHeartBeat) {
                    updateLightBar();
                }
                // Task 3
                if(currentPlan != null) {
                    synchronized(this.currentPlan) {
                        if(currentPlan != null) {
                            boolean isCurrentPlanTimeout = ((System.currentTimeMillis() - this.currentPlan.planStartTime) > PlatooningPlugin.NEGOTIATION_TIMEOUT);
                            if(isCurrentPlanTimeout) {
                                log.info("Give up current on waiting plan with planId: " + this.currentPlan.planId);
                                this.currentPlan = null;
                            }    
                        }
                    }
                }
                // Task 4
                boolean hasFollower = plugin.platoonManager.getPlatooningSize() != 1; 
                if(hasFollower) {
                    MobilityOperation statusOperation = plugin.mobilityOperationPublisher.newMessage();
                    composeMobilityOperation(statusOperation, "STATUS");
                    plugin.mobilityOperationPublisher.publish(statusOperation);
                    log.debug("Published platoon STATUS operation message");
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
        return "PlatoonLeaderState";
    }

    private boolean isVehicleRightInFront(String rearVehicleBsmId) {
        //log.debug("The current dtd headway from GPS is " + (platoonRearDtd - currentHostDtd) + " m");
        return true;
    }
    
    /**
     * Helper function to update the light bar
     */
    private void updateLightBar() {
        // Set the light bar flashing if we are a leader with a follower
        if (plugin.platoonManager.getTotalPlatooningSize() > 1) {
            plugin.setLightBarStatus(IndicatorStatus.FLASH);
        } else {
            // Turn off the light bar
            plugin.setLightBarStatus(IndicatorStatus.OFF);
        }
    }

}
