package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.util.UUID;

import cav_msgs.MobilityHeader;
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
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlanStatus;
import gov.dot.fhwa.saxton.carma.plugins.platooning.PlatoonPlan.PlatooningPlanType;

/**
 * The SingleVehiclePlatoonState is a state which platooning algorithm is enabled
 * on the current trajectory or the next trajectory but it did not actually lead any vehicles.
 * This state will transit to Standby state when the algorithm is disabled on the next trajectory;
 * it will transit to CandidateFollower state if it is trying to join another platoon in front;
 * it will also transit to LeaderWaiting state if it is waiting on another vehicle to join at its rear.
 * In this state, it will not active plan any maneuvers but it will handle any JOIN request,
 * actively look for an available platoon in front and sends out heart beat platoon message to inform other CAVs.
 */
public class SingleVehiclePlatoonState implements IPlatooningState {

    protected PlatooningPlugin     plugin;
    protected ILogger              log;
    protected PluginServiceLocator pluginServiceLocator;
    private   PlatoonPlan          currentPlan;

    public SingleVehiclePlatoonState(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
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
            plugin.setState(new StandbyState(plugin, log, pluginServiceLocator));
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
            MobilityHeader msgHeader = msg.getHeader();
            String params = msg.getStrategyParams();
            log.debug("Receive mobility request to join a platoon from " + msgHeader.getSenderId() + " and PlanId = " + msgHeader.getPlanId());
            log.debug("The strategy parameters are " + params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,MAX_ACCEL:xx"
            int applicantSize = Integer.parseInt(params.split(",")[0].split(":")[1]);
            // check if we have enough room for that applicant
            boolean hasEnoughRoomInPlatoon = applicantSize + 1 <= plugin.getMaxPlatoonSize();
            if(hasEnoughRoomInPlatoon) {
                log.debug("The current platoon has enough room for the applicant with size " + applicantSize);
                double currentDtd = pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
                // TODO need trajectory convert provide the functionality to convert from ECEF point to downtrack distance  
                cav_msgs.Trajectory dummyTraj = plugin.getMobilityRequestPublisher().newMessage().getTrajectory();
                dummyTraj.setLocation(msg.getLocation());
                double applicantCurrentDtd = pluginServiceLocator.getTrajectoryConverter().messageToPath(dummyTraj).get(0).getDowntrack();
                log.debug("The current vehicle and applicant have a gap with length " + (currentDtd - applicantCurrentDtd));
                // check if the applicant can join immediately without any acceleration
                boolean isDistanceCloseEnough = (currentDtd - applicantCurrentDtd) <= plugin.getDesiredJoinDistance();
                if(isDistanceCloseEnough) {
                    log.debug("The applicant is close enough and it can join without accelerarion. Changing to LeaderWaitingState.");
                    plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator));
                    return MobilityRequestResponse.ACK;
                } else {
                    log.debug("The applicant is not close enough. Calculating if it can join in a reasonable time");
                    // Assume the applicant has the same speed with us and is lower than speed limit
                    double currentSpeed = plugin.getManeuverInputs().getCurrentSpeed();
                    double currentSpeedLimit = pluginServiceLocator.getRouteService().getSpeedLimitAtLocation(currentDtd).getLimit();
                    double applicantMaxAccel = Double.parseDouble(params.split(",")[1].split(":")[1]);
                    // Assume the applicant can speed up to the speed limit
                    double applicantSpeedUpTime = (currentSpeedLimit - currentSpeed) / applicantMaxAccel;
                    // If we ignore the gap change during applicant acceleration,
                    // we can calculate the time the applicant will take to reach the desired join distance
                    double timeToCatchUp = (currentDtd - applicantCurrentDtd - plugin.getDesiredJoinDistance()) / (currentSpeedLimit - currentSpeed);
                    // Assume there is no vehicle lag time, we now know the total time need for the applicant to join
                    double totalTimeNeeded = applicantSpeedUpTime + timeToCatchUp;
                    // check if it is a reasonable time
                    boolean isTotalTimeReasonable = totalTimeNeeded < plugin.getMaxJoinTime();
                    log.debug("The host vehicle current speed is " + currentSpeed + " but the speed limit is " + currentSpeedLimit);
                    log.debug("The applicant max accel is " + applicantMaxAccel + " so it need " + applicantSpeedUpTime + "to speed up");
                    log.debug("The applicant also needs " + timeToCatchUp + " to close the gap. So the total time is " + totalTimeNeeded);
                    if(isTotalTimeReasonable) {
                        log.debug("The total time for applicant joining is reasonable. Changing to LeaderWaitingState.");
                        plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator));
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
        // In the current state, we only care about the INFO heart beat operation message if we are not currently in a negotiation
        boolean isPlatoonInfoMsg = strategyParams.startsWith("INFO");
        boolean isNotInNegotiation = this.currentPlan == null;
        if(isPlatoonInfoMsg && isNotInNegotiation) {
            // For INFO params, the string format is INFO|LEADER:xx,ECEFx:xx,ECEFy:xx,ECEFz:xx
            // TODO need trajectory convert provide the functionality to convert from ECEF point to downtrack distance  
            cav_msgs.Trajectory dummyTraj = plugin.getMobilityRequestPublisher().newMessage().getTrajectory();
            dummyTraj.getLocation().setEcefX(Integer.parseInt(strategyParams.split(",")[1].split(":")[1]));
            dummyTraj.getLocation().setEcefY(Integer.parseInt(strategyParams.split(",")[2].split(":")[1]));
            dummyTraj.getLocation().setEcefZ(Integer.parseInt(strategyParams.split(",")[3].split(":")[1]));
            double platoonDtd = pluginServiceLocator.getTrajectoryConverter().messageToPath(dummyTraj).get(0).getDowntrack();
            boolean isInFrontOfHostVehicle = platoonDtd > pluginServiceLocator.getRouteService().getCurrentDowntrackDistance();
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
                request.getPlanType().setType((byte) 3);
                request.setStrategy(plugin.MOBILITY_STRATEGY);
                request.setStrategyParams(String.format("SIZE:1,MAX_ACCEL:%.2f", plugin.getMaxAccel()));
                // TODO need to populate the urgency
                request.setUrgency((short) 50);
                this.currentPlan = new PlatoonPlan(System.currentTimeMillis(), planId, PlanStatus.WAITING_ON_RESPONSE, PlatooningPlanType.JOIN_FROM_REAR);
                plugin.getMobilityRequestPublisher().publish(request);
            } else {
                log.debug("Ignore platoon behind the host vehicle with platoon id: " + msg.getPlanId());
            }
        } else {
            log.debug("Receive operation message but ignore it isPlatoonInfoMsg = " + isPlatoonInfoMsg + " & isNotInNegotiation = " + isNotInNegotiation);
        }
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
        return "SingleVehiclePlatoonState";
    }
}
