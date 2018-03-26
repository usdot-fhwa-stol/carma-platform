package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.UUID;

import cav_msgs.MobilityHeader;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlanType;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRequestResponse;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlanStatus;
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlatooningPlanType;

/**
 * The PlatoonLeaderState is a state which platooning algorithm is enabled
 * on the current trajectory or the next trajectory no matter whether it actually lead vehicles or not.
 * This state will transit to Standby state when the algorithm is disabled on the next trajectory;
 * it will transit to CandidateFollower state if it is trying to join another platoon in front;
 * it will also transit to LeaderWaiting state if it is waiting on another vehicle behind him to join at its platoon rear.
 * In this state, it will not active plan any maneuvers but it will handle any JOIN request,
 * actively look for an available platoon in front and sends out heart beat platoon message to inform its existence.
 */
public class PlatoonLeaderState implements IPlatooningState {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   PlatoonPlan          currentPlan;
    private   long                 lastHeartBeatTime = 0;
    private   double               speedUpTime       = 0.0;

    public PlatoonLeaderState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin = plugin;
        this.log = log;
        this.pluginServiceLocator = pluginServiceLocator;
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        RouteService rs = pluginServiceLocator.getRouteService();
        TrajectoryPlanningResponse tpr = new TrajectoryPlanningResponse();
        // If there is no platooning window, we set plugin to standby state
        if(!rs.isAlgorithmEnabledInRange(traj.getStartLocation(), traj.getEndLocation(), plugin.PLATOONING_FLAG)) {
            log.info("There is no platoon window in " + traj.toString() + " . Change back to standby state from " + this.toString());
            // TODO send out mobility operation message to inform LEAVE and delegate the leader job
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
        } else {
            // as a leader, the actual plan job is done by the default cruising plug-in
            log.debug("Not insert any maneuvers in trajectory at LeaderWaitingState in " + traj.toString());
        }
        return tpr;
    }

    @Override
    public MobilityRequestResponse onMobilityRequestMessgae(MobilityRequest msg) {
        // If we received a JOIN request, we decide whether we allow that CAV to join
        // If yes, we need to change to leader waiting state 
        if(msg.getPlanType().getType() == (PlanType.JOIN_PLATOON_AT_REAR)) {
            // We need to check the size limitation on current platoon based on the leader's parameters
            // and we also need to calculate how long that vehicle can be in a reasonable distance to actually join
            // TODO To evaluate the JOIN conditions, we assume the applicant is in the same lane with us for now
            MobilityHeader msgHeader = msg.getHeader();
            String params = msg.getStrategyParams();
            String applicantId = msgHeader.getSenderId();
            log.debug("Receive mobility JOIN request from " + applicantId + " and PlanId = " + msgHeader.getPlanId());
            log.debug("The strategy parameters are " + params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,MAX_ACCEL:xx"
            int applicantSize = Integer.parseInt(params.split(",")[0].split(":")[1]);
            // check if we have enough room for that applicant
            int currentNumberOfFollower = plugin.getPlatoonManager().getPlatooningSize();
            boolean hasEnoughRoomInPlatoon = applicantSize + currentNumberOfFollower + 1 <= plugin.getMaxPlatoonSize();
            if(hasEnoughRoomInPlatoon) {
                log.debug("The current platoon has enough room for the applicant with size " + applicantSize);
                double currentRearDtd;
                if(currentNumberOfFollower == 0) {
                    currentRearDtd = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
                    log.debug("The current platoon only have the leader itself, so the platoon rear dtd is " + currentRearDtd);
                } else {
                    currentRearDtd = plugin.getPlatoonManager().platoon.get(currentNumberOfFollower - 1).getVehiclePosition();
                    log.debug("The current platoon has " + currentNumberOfFollower + " followers, so the platoon rear dtd is " + currentRearDtd);
                }
                // TODO need trajectory convert provide the functionality to convert from ECEF point to downtrack distance  
                cav_msgs.Trajectory dummyTraj = plugin.getMobilityRequestPublisher().newMessage().getTrajectory();
                dummyTraj.setLocation(msg.getLocation());
                double applicantCurrentDtd = pluginServiceLocator.getTrajectoryConverter().messageToPath(dummyTraj).get(0).getDowntrack();
                log.debug("The current vehicle and applicant have a gap with length " + (currentRearDtd - applicantCurrentDtd));
                // check if the applicant can join immediately without any acceleration
                boolean isDistanceCloseEnough = (currentRearDtd - applicantCurrentDtd) <= plugin.getDesiredJoinDistance();
                if(isDistanceCloseEnough) {
                    log.debug("The applicant is close enough and it can join without accelerarion.");
                    log.debug("Changing to LeaderWaitingState and waiting for " + msg.getHeader().getSenderId());
                    plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, msg.getHeader().getSenderId()));
                    return MobilityRequestResponse.ACK;
                } else {
                    log.debug("The applicant is not close enough. Calculating if it can join in a reasonable time");
                    // Assume the applicant has the same speed with us and is lower than speed limit
                    double currentSpeed = plugin.getManeuverInputs().getCurrentSpeed();
                    double currentSpeedLimit = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentRearDtd).getLimit();
                    double applicantMaxAccel = Double.parseDouble(params.split(",")[1].split(":")[1]);
                    // Assume the applicant can speed up to the speed limit
                    double applicantSpeedUpTime = (currentSpeedLimit - currentSpeed) / applicantMaxAccel;
                    // If we ignore the gap change during applicant acceleration,
                    // we can calculate the time the applicant will take to reach the desired join distance
                    double timeToCatchUp = (currentRearDtd - applicantCurrentDtd - plugin.getDesiredJoinDistance()) / (currentSpeedLimit - currentSpeed);
                    // Assume there is no vehicle lag time, we now know the total time need for the applicant to join
                    double totalTimeNeeded = applicantSpeedUpTime + timeToCatchUp;
                    // check if it is a reasonable time
                    boolean isTotalTimeReasonable = totalTimeNeeded < plugin.getMaxJoinTime();
                    log.debug("The host vehicle current speed is " + currentSpeed + " but the speed limit is " + currentSpeedLimit);
                    log.debug("The applicant max accel is " + applicantMaxAccel + " so it need " + applicantSpeedUpTime + "to speed up");
                    log.debug("The applicant also needs " + timeToCatchUp + " to close the gap. So the total time is " + totalTimeNeeded);
                    if(isTotalTimeReasonable) {
                        log.debug("The total time for applicant joining is reasonable.");
                        log.debug("Changing to LeaderWaitingState and waiting for " + msg.getHeader().getSenderId());
                        plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, msg.getHeader().getSenderId()));
                        return MobilityRequestResponse.ACK;
                    } else {
                        log.debug("The total time for applicant joining is unreasonable. Sending NACK to this request");
                        return MobilityRequestResponse.NACK;
                    }
                }
            } else {
                return MobilityRequestResponse.NACK;
            }
        } else {
            return MobilityRequestResponse.NO_RESPONSE;
        }
    }

    @Override
    public void onMobilityOperationMessage(MobilityOperation msg) {
        String strategyParams = msg.getStrategyParams();
        // In the current state, we care about the INFO heart beat operation message if we are not currently in a negotiation,
        // and also we need to care about operation from members in the current platoon
        boolean isPlatoonInfoMsg = strategyParams.startsWith("INFO");
        boolean isPlatoonStatusMsg = strategyParams.startsWith("STATUS");
        boolean isNotInNegotiation = this.currentPlan == null;
        if(isPlatoonInfoMsg && isNotInNegotiation) {
            // For INFO params, the string format is INFO|LEADER:xx,ECEFx:xx,ECEFy:xx,ECEFz:xx,SPEED:xx
            // TODO need trajectory convert provide the functionality to convert from ECEF point to downtrack distance  
            cav_msgs.Trajectory dummyTraj = plugin.getMobilityRequestPublisher().newMessage().getTrajectory();
            dummyTraj.getLocation().setEcefX(Integer.parseInt(strategyParams.split(",")[1].split(":")[1]));
            dummyTraj.getLocation().setEcefY(Integer.parseInt(strategyParams.split(",")[2].split(":")[1]));
            dummyTraj.getLocation().setEcefZ(Integer.parseInt(strategyParams.split(",")[3].split(":")[1]));
            double platoonRearDtd = pluginServiceLocator.getTrajectoryConverter().messageToPath(dummyTraj).get(0).getDowntrack();
            // calculate how much time the host should accelerate to close the gap
            double platoonSpeed = Double.parseDouble(strategyParams.split(",")[4].split(":")[1]);
            double currentHostDtd = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
            double hostMaxSpeed = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentHostDtd).getLimit();
            this.speedUpTime = (platoonRearDtd - currentHostDtd) / (hostMaxSpeed - platoonSpeed);
            boolean isInFrontOfHostVehicle = platoonRearDtd > pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
            if(isInFrontOfHostVehicle) {
                MobilityRequest request = plugin.getMobilityRequestPublisher().newMessage();
                String planId = UUID.randomUUID().toString();
                request.getHeader().setPlanId(planId);
                String leaderId = strategyParams.split(",")[0].split(":")[1];
                request.getHeader().setRecipientId(leaderId);
                // TODO need to have a easy way to get bsmId in plugin
                request.getHeader().setSenderBsmId("FFFFFFFF");
                request.getHeader().setSenderId(pluginServiceLocator.getMobilityRouter().getHostMobilityId());
                request.getHeader().setTimestamp(System.currentTimeMillis());
                // TODO need to have a easy way to get current location in ECEF
                request.getPlanType().setType(PlanType.JOIN_PLATOON_AT_REAR);
                request.setStrategy(plugin.MOBILITY_STRATEGY);
                request.setStrategyParams(String.format("SIZE:1,MAX_ACCEL:%.2f", plugin.getMaxAccel()));
                // TODO need to populate the urgency later
                request.setUrgency((short) 50);
                this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, PlanStatus.WAITING_ON_RESPONSE, PlatooningPlanType.JOIN_FROM_REAR);
                plugin.getMobilityRequestPublisher().publish(request);
            } else {
                log.debug("Ignore platoon behind the host vehicle with platoon id: " + msg.getHeader().getPlanId());
            }
        } else if(isPlatoonStatusMsg) {
            // if it is platoon status message, the params string is in format:
            // STATUS|CMDSPEED:5.0, DOWNTRACK:100.0, SPEED:5.0
            // we only care about platoon status message when the strategy id matches
            if(plugin.getPlatoonManager().getCurrentPlatoonID().equals(msg.getStrategyId())) {
                // TODO maybe we do not need to updates all infomations on every vehicles
                String vehicleID = msg.getHeader().getSenderId();
                String statusParams    = strategyParams.split("|")[1];
                log.info("Receive operation message from our platoon from vehicle: " + vehicleID);
                plugin.getPlatoonManager().memberUpdates(vehicleID, statusParams);
            }
        } else {
            log.debug("Receive operation message but ignore it because isPlatoonInfoMsg = " + isPlatoonInfoMsg 
                    + ", isNotInNegotiation = " + isNotInNegotiation + " and isPlatoonStatusMsg = " + isPlatoonStatusMsg);
        }
    }

    @Override
    public void onMobilityResponseMessage(MobilityResponse msg) {
        // Only care the response message for the plan on which we are waiting
        if(this.currentPlan != null && this.currentPlan.status == PlanStatus.WAITING_ON_RESPONSE &&
           this.currentPlan.planId.equals(msg.getHeader().getPlanId())) {
            if(msg.getIsAccepted()) {
                log.debug("Received positive response for plan id = " + this.currentPlan.planId);
                log.debug("Change to CandidateFollower state and notify trajectory failure in order to replan");
                // change to candidate follower state and request a new plan to catch with the front vehicle
                plugin.setState(new CandidateFollowerState(plugin, log, pluginServiceLocator, this.speedUpTime));
                pluginServiceLocator.getArbitratorService().notifyTrajectoryFailure();
            } else {
                log.debug("Received negative response for plan id = " + this.currentPlan.planId);
                // forget about the previous plan
                this.currentPlan = null;
            }
        }
    }

    @Override
    public void run() {
        // This is a loop which is safe to interrupt
        // This loop does three things:
        // 1. Send out heart beat mobility operation INFO message every 3 seconds
        // 2. Remove current plan if we wait for long enough time
        // 3. Publish operation status every 100 milliseconds if necessary
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                // Publish heart beat message if the current platoon is not full
                if(tsStart - lastHeartBeatTime >= plugin.getOperationInfoIntervalLength() &&
                    plugin.getPlatoonManager().getPlatooningSize() + 1 < plugin.getMaxPlatoonSize()) {
                    MobilityOperation infoOperation = plugin.getMobilityOperationPublisher().newMessage();
                    composeMobilityOperation(infoOperation, "INFO");
                    plugin.getMobilityOperationPublisher().publish(infoOperation);
                    log.debug("Published heart beat message with parameters: " + infoOperation.getStrategyParams());
                }
                // Remove old timeout plan
                if(this.currentPlan != null && this.currentPlan.status == PlanStatus.WAITING_ON_RESPONSE) {
                    long waitTime = System.currentTimeMillis() - this.currentPlan.planStartTime;
                    if(waitTime > plugin.getShortNegotiationTimeout()) {
                        this.currentPlan = null;
                        log.info("Give up current on waiting plan with planId: " + this.currentPlan.planId);
                    }
                }
                // Publish operation status
                if(plugin.getPlatoonManager().getPlatooningSize() != 0) {
                    MobilityOperation statusOperation = plugin.getMobilityOperationPublisher().newMessage();
                    composeMobilityOperation(statusOperation, "STATUS");
                    plugin.getMobilityOperationPublisher().publish(statusOperation);
                    log.debug("Published STATUS operation message with parameters: " + statusOperation.getStrategyParams());
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
        if(type.equals("INFO")) {
            //For INFO params, the string format is INFO|LEADER:xx,ECEFx:xx,ECEFy:xx,ECEFz:xx
            // TODO need to have a easy way to get current location in ECEF
            msg.setStrategyParams("INFO|LEADER:" + hostStaticId + ",ECEFx:0.0,ECEFy:0.0,ECEFz:0.0");
        } else if(type.equals("STATUS")) {
            // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
            SpeedAccel lastCmdSpeed = plugin.getCmdSpeedSub().getLastMessage();
            double cmdSpeed = lastCmdSpeed == null ? 0.0 : lastCmdSpeed.getSpeed();
            double downtrackDistance = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
            double currentSpeed = pluginServiceLocator.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
            String params = String.format("STATUS|CMDSPEED:%.2f,DOWNTRACK:%.2f,SPEED:%.2f", cmdSpeed, downtrackDistance, currentSpeed);
            msg.setStrategyParams(params);
        } else {
            log.error("UNKNOW strategy param string!!!");
            msg.setStrategyParams("");
        }
    }
    
}
