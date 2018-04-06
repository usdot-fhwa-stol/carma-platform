package gov.dot.fhwa.saxton.carma.plugins.platooning;

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
public class PlatoonLeaderState implements IPlatooningState {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   PlatoonPlan          currentPlan;
    // This speedUpTime field is used when it sends out a JOIN request and is prepared to transit to CandidateFollower
    // state. This field describes how much time the CandidateFollower state need to keep at the speed limit
    // after speed-up, in order to close the gap with the rear vehicle in the target platoon in front of it.
    private   double               speedUpTime           = 0.0;
    private   long                 lastHeartBeatTime     = 0;
    private   String               potentialNewPlatoonId = "";

    public PlatoonLeaderState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
        // Update the light bar
        updateLightBar();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is a platooning window
        if(rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
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
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,ACCEL:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            // TODO And we need trajectory convert provide the functionality to convert from ECEF point to down track distance easily
            int applicantSize = Integer.parseInt(params.split(",")[0].split(":")[1]);
            double applicantMaxAccel = Double.parseDouble(params.split(",")[1].split(":")[1]);
            double applicantCurrentDtd = Double.parseDouble(params.split(",")[2].split(":")[1]);
            // Check if we have enough room for that applicant
            int currentNumberOfFollower = plugin.getPlatoonManager().getPlatooningSize();
            boolean hasEnoughRoomInPlatoon = applicantSize + currentNumberOfFollower + 1 <= plugin.getMaxPlatoonSize();
            if(hasEnoughRoomInPlatoon) {
                log.debug("The current platoon has enough room for the applicant with size " + applicantSize);
                double currentRearDtd = plugin.getPlatoonManager().getPlatoonRearDowntrackDistance();
                log.debug("The current platoon rear dtd is " + currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd;
                log.debug("The gap between current platoon rear and applicant is " + currentGap + "meters");
                if(currentGap < 0) {
                    log.warn("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse.NACK;
                }
                // Check if the applicant can join immediately without any accelerations
                boolean isDistanceCloseEnough = currentGap <= plugin.getDesiredJoinDistance();
                if(isDistanceCloseEnough) {
                    log.debug("The applicant is close enough and it can join without accelerarion");
                    log.debug("Changing to LeaderWaitingState and waiting for " + msg.getHeader().getSenderId() + " to join");
                    plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse.ACK;
                } else {
                    log.debug("The applicant is not close enough. Calculating whether it can join in a reasonable time");
                    // Assume the applicant has the same speed with us and is lower than speed limit
                    double currentPlatoonSpeed = plugin.getManeuverInputs().getCurrentSpeed();
                    double currentSpeedLimit = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentRearDtd).getLimit();
                    double speedDiff = currentSpeedLimit - currentPlatoonSpeed;
                    if(speedDiff == 0) {
                        log.warn("Speed Diff is 0 such that the applicant can never catch up with us without speed limit violation. NACK");
                        return MobilityRequestResponse.NACK;
                    } else if(speedDiff < 0) {
                        log.warn("Our current speed is violate the local speed limit!!! NACK");
                        return MobilityRequestResponse.NACK;
                    } else {
                        log.debug("Speed difference between speed limit and platoon speed is " + speedDiff);
                    }
                    // Assume the applicant can speed up to the speed limit using maxAccel
                    double applicantSpeedUpTime = Math.max((speedDiff) / applicantMaxAccel, 0);
                    // If we ignore the gap change during applicant acceleration, then we can
                    // calculate the time the applicant will take to reach the desired join distance
                    double timeToCatchUp = Math.max((currentGap - plugin.getDesiredJoinDistance()) / (speedDiff), 0);
                    // We need to plus the vehicle lag time
                    double vehicleResponseLag = plugin.getManeuverInputs().getResponseLag();
                    double totalTimeNeeded = applicantSpeedUpTime + timeToCatchUp + vehicleResponseLag;
                    // Check if it is a reasonable total time
                    boolean isTotalTimeReasonable = totalTimeNeeded <= plugin.getMaxJoinTime();
                    log.debug("The host vehicle current speed is " + currentPlatoonSpeed + " but the speed limit is " + currentSpeedLimit);
                    log.debug("The applicant max accel is " + applicantMaxAccel + " so it need " + applicantSpeedUpTime + "to speed up");
                    log.debug("The applicant also needs " + timeToCatchUp + " to close the gap and " + vehicleResponseLag + " response lag time");
                    log.debug("So the total time is " + totalTimeNeeded + " and isTotalTimeReasonable = " + isTotalTimeReasonable);
                    if(isTotalTimeReasonable) {
                        log.debug("Changing to LeaderWaitingState and waiting for " + applicantId + " to join");
                        plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                        return MobilityRequestResponse.ACK;
                    } else {
                        log.debug("The total time for applicant joining is unreasonable. Sending NACK to this request");
                        return MobilityRequestResponse.NACK;
                    }
                }
            } else {
                log.debug("The current platoon does not have enough room to applicant with size " + applicantSize + " and NACK");
                return MobilityRequestResponse.NACK;
            }
        } else {
            log.debug("Received mobility request with type " + msg.getPlanType().getType() + " but ignored.");
            return MobilityRequestResponse.NO_RESPONSE;
        }
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        String strategyParams = msg.getStrategyParams();
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in the current platoon
        boolean isPlatoonInfoMsg = strategyParams.startsWith(plugin.OPERATION_INFO_TYPE);
        boolean isPlatoonStatusMsg = strategyParams.startsWith(plugin.OPERATION_STATUS_TYPE);
        boolean isNotInNegotiation = (this.currentPlan == null);
        if(isPlatoonInfoMsg && isNotInNegotiation) {
            // For INFO params, the string format is INFO|LEADER:xx,REAR_DTD:xx,SPEED:xx
            // TODO In future, we should remove downtrack distance from this string and send XYZ location in ECEF
            String leaderId = strategyParams.split(",")[0].split(":")[1]; 
            double platoonRearDtd = Double.parseDouble(strategyParams.split(",")[1].split(":")[1]);
            double platoonSpeed = Double.parseDouble(strategyParams.split(",")[2].split(":")[1]);
            double currentHostDtd = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
            boolean isInFrontOfTheHostVehicle = platoonRearDtd > currentHostDtd;
            if(isInFrontOfTheHostVehicle) {
                log.debug("Found a platoon with id = " + msg.getHeader().getPlanId() + " in front of us.");
                // Calculate how much time the host should accelerate to close the gap
                double hostMaxSpeed = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentHostDtd).getLimit();
                // Whether we can join by speed up
                if(hostMaxSpeed <= platoonSpeed) {
                    log.debug("The local speed limit " + hostMaxSpeed + "is less or equal with the platoon speed. Unable to join");
                    return;
                }
                if(platoonRearDtd - currentHostDtd - plugin.getDesiredJoinDistance() <= 0) {
                    log.debug("We do not need to speed up in order to join.");
                } else {
                    speedUpTime = (platoonRearDtd - currentHostDtd - plugin.getDesiredJoinDistance()) / (hostMaxSpeed - platoonSpeed);
                    log.debug("The speed up time we need to close the gap is roughly " + speedUpTime);
                }
                // Compose a mobility request to publish a JOIN request
                MobilityRequest request = plugin.getMobilityRequestPublisher().newMessage();
                String planId = UUID.randomUUID().toString();
                request.getHeader().setPlanId(planId);
                request.getHeader().setRecipientId(leaderId);
                // TODO Need to have a easy way to get bsmId from plugin
                request.getHeader().setSenderBsmId("FFFFFFFF");
                request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                request.getHeader().setTimestamp(System.currentTimeMillis());
                // TODO Need to have a easy way to get current XYZ location in ECEF
                request.getLocation().setEcefX(0);
                request.getLocation().setEcefY(0);
                request.getLocation().setEcefZ(0);
                request.getLocation().setTimestamp(System.currentTimeMillis());
                request.getPlanType().setType(PlanType.JOIN_PLATOON_AT_REAR);
                request.setStrategy(plugin.MOBILITY_STRATEGY);
                String strategyParamsString = String.format(plugin.JOIN_AT_REAR_PARAMS,
                                                      plugin.getPlatoonManager().getPlatooningSize() + 1,
                                                      plugin.getMaxAccel(), currentHostDtd);
                request.setStrategyParams(strategyParamsString);
                // TODO Need to populate the urgency later
                request.setUrgency((short) 50);
                this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, leaderId);
                log.debug("Publishing request to leader " + leaderId + " with params " + strategyParamsString);
                plugin.getMobilityRequestPublisher().publish(request);
                this.potentialNewPlatoonId = msg.getHeader().getPlanId();
            } else {
                log.debug("Ignore platoon behind the host vehicle with platoon id: " + msg.getHeader().getPlanId());
            }
        } else if(isPlatoonStatusMsg) {
            // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
            // The platoonManager will ignore it if it is not from our platoon
            String vehicleID = msg.getHeader().getSenderId();
            String statusParams = strategyParams.substring(plugin.OPERATION_STATUS_TYPE.length() + 1);
            log.info("Receive operation status message from vehicle: " + vehicleID + " with params: " + statusParams);
            plugin.getPlatoonManager().memberUpdates(vehicleID, msg.getHeader().getPlanId(), statusParams);
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
                if(this.currentPlan.planId.equals(msg.getHeader().getPlanId()) &&
                   this.currentPlan.peerId.equals(msg.getHeader().getSenderId())) {
                    if(msg.getIsAccepted()) {
                        log.debug("Received positive response for plan id = " + this.currentPlan.planId);
                        log.debug("Change to CandidateFollower state and notify trajectory failure in order to replan");
                        // Change to candidate follower state and request a new plan to catch up with the front platoon
                        plugin.setState(new CandidateFollowerState(plugin, log, pluginServiceLocator,
                                        speedUpTime, currentPlan.peerId, potentialNewPlatoonId));
                        pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
                    } else {
                        log.debug("Received negative response for plan id = " + this.currentPlan.planId);
                        // Forget about the previous plan totally
                        this.currentPlan = null;
                    }
                }
            }
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
                boolean isTimeForHeartBeat = tsStart - lastHeartBeatTime >= plugin.getOperationInfoIntervalLength();
                boolean isPlatoonNotFull = plugin.getPlatoonManager().getPlatooningSize() + 1 < plugin.getMaxPlatoonSize(); 
                if(isTimeForHeartBeat && isPlatoonNotFull) {
                    MobilityOperation infoOperation = plugin.getMobilityOperationPublisher().newMessage();
                    composeMobilityOperation(infoOperation, "INFO");
                    plugin.getMobilityOperationPublisher().publish(infoOperation);
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
                        boolean isCurrentPlanTimeout = ((System.currentTimeMillis() - this.currentPlan.planStartTime) > plugin.getShortNegotiationTimeout());
                        if(isCurrentPlanTimeout) {
                            log.info("Give up current on waiting plan with planId: " + this.currentPlan.planId);
                            this.currentPlan = null;
                        }
                    }
                }
                // Task 4
                boolean hasFollower = plugin.getPlatoonManager().getPlatooningSize() != 0; 
                if(hasFollower) {
                    MobilityOperation statusOperation = plugin.getMobilityOperationPublisher().newMessage();
                    composeMobilityOperation(statusOperation, "STATUS");
                    plugin.getMobilityOperationPublisher().publish(statusOperation);
                    log.debug("Published platoon STATUS operation message");
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
        return "PlatoonLeaderState";
    }
    
    // This method compose mobility operation INFO/STATUS message
    private void composeMobilityOperation(MobilityOperation msg, String type) {
        msg.getHeader().setPlanId(plugin.getPlatoonManager().getCurrentPlatoonID());
        // All platoon mobility operation message is just for broadcast
        msg.getHeader().setRecipientId("");
        // TODO Need to have a easy way to get bsmId from plugin
        msg.getHeader().setSenderBsmId("FFFFFFFF");
        String hostStaticId = pluginServiceLocator.getMobilityRouter().getHostMobilityId();
        msg.getHeader().setSenderId(hostStaticId);
        msg.getHeader().setTimestamp(System.currentTimeMillis());
        msg.setStrategy(plugin.MOBILITY_STRATEGY);
        if(type.equals(plugin.OPERATION_INFO_TYPE)) {
            // For INFO params, the string format is INFO|LEADER:xx,REAR_DTD:xx,SPEED:xx
            String infoParams = String.format(plugin.OPERATION_INFO_PARAMS,
                                hostStaticId, plugin.getPlatoonManager().getPlatoonRearDowntrackDistance(),
                                pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
            msg.setStrategyParams(infoParams);
        } else if(type.equals(plugin.OPERATION_STATUS_TYPE)) {
            // TODO Maneuver planner from plugin service locator may need to provide this data directly 
            SpeedAccel lastCmdSpeedObject = plugin.getCmdSpeedSub().getLastMessage();
            double cmdSpeed = lastCmdSpeedObject == null ? 0.0 : lastCmdSpeedObject.getSpeed();
            // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
            String statusParams = String.format(plugin.OPERATION_STATUS_PARAMS,
                                                cmdSpeed, pluginServiceLocator.getRouteService().getCurrentDowntrackDistance(),
                                                pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
            msg.setStrategyParams(statusParams);
        } else {
            log.error("UNKNOW strategy param string!!!");
            msg.setStrategyParams("");
        }
        log.debug("Composed a mobility operation message with params " + msg.getStrategyParams());
    }

    /**
     * Helper function to update the light bar
     */
    private void updateLightBar() {
        // Set the light bar flashing if we are a leader with a follower
        if (plugin.getPlatoonManager().getPlatooningSize() != 0) {
            plugin.setLightBarStatus(IndicatorStatus.FLASH);
        } else {
            // Turn off the light bar
            plugin.setLightBarStatus(IndicatorStatus.OFF);
        }
    }

}
