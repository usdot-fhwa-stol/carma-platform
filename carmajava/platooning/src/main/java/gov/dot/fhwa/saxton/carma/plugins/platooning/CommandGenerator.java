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

import java.util.Optional;

import com.google.common.util.concurrent.AtomicDouble;

import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.signals.Filter;
import gov.dot.fhwa.saxton.carma.guidance.signals.PidController;
import gov.dot.fhwa.saxton.carma.guidance.signals.Pipeline;
import gov.dot.fhwa.saxton.carma.guidance.signals.Signal;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.SpeedLimit;

/**
 * This class generates speed commands based on the latest information from plugin platoon list.
 */
public class CommandGenerator implements Runnable, IPlatooningCommandInputs {
    
    protected static long CMD_TIMESTEP = 100;
    
    protected PlatooningPlugin plugin_;
    protected PluginServiceLocator pluginServiceLocator_;
    protected ILogger log_;
    protected Filter<Double> distanceGapController_;
    protected Pipeline<Double> speedController_;
    protected double desiredGap_ = 0.0;
    protected double adjustmentCap = 10.0;
    protected AtomicDouble speedCmd_ = new AtomicDouble(0.0);
    protected Optional<Double> lastCmdSpeed = Optional.empty();
    
    private boolean enableMaxAccelFilter;
    private boolean enableMaxAdjustmentFilter;
    private boolean enableLocalSpeedLimitFilter;
    
    @SuppressWarnings("unchecked")
    public CommandGenerator(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin_ = plugin;
        this.pluginServiceLocator_ = pluginServiceLocator;
        this.log_ = log;
        this.distanceGapController_ = new PidController(plugin_.kpPID, plugin_.kiPID, plugin_.kdPID, plugin_.standStillHeadway);
        if(plugin_.integratorMaxCap > plugin_.integratorMinCap) {
            ((PidController) this.distanceGapController_).setIntegratorRange(plugin_.integratorMinCap, plugin_.integratorMaxCap);
        }
        this.speedController_ = new Pipeline<Double>(distanceGapController_);
        this.adjustmentCap = Math.max(0, plugin.cmdSpeedMaxAdjustment);
        this.enableMaxAccelFilter = plugin.maxAccelCapEnabled;
        this.enableLocalSpeedLimitFilter = plugin.speedLimitCapEnabled;
        this.enableMaxAdjustmentFilter = plugin.leaderSpeedCapEnabled;
    }

    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                generateSpeed((double) tsStart);
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(CMD_TIMESTEP - (tsEnd - tsStart), 0);
                Thread.sleep(sleepDuration);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    @Override
    public double getLastSpeedCommand() {
        return speedCmd_.get();
    }
    
    @Override
    public double getMaxAccelLimit() {
        return plugin_.maxAccel;
    }
    
    protected void generateSpeed(double timeStamp) {
        // Update speed commands based on the list of platoon members
        PlatoonMember leader = null;
        if(plugin_.platoonManager != null) {
            leader = plugin_.platoonManager.getLeader();
        }
        if(leader != null) {
            double controllerOutput = 0.0;
            // for truck platooning, we decide to use radar to maintain a time gap between vehicles
            if(plugin_.algorithmType == PlatooningPlugin.LPF_ALGORITHM) {
                double currentGap = plugin_.getManeuverInputs().getDistanceToFrontVehicle();
                // if there is an error from radar reading
                if(!Double.isFinite(currentGap)) {
                    log_.warn("We lost the track of front vehicle. Using leader command speed");
                } else {
                    distanceGapController_.changeSetpoint(plugin_.desiredTimeGap * plugin_.getManeuverInputs().getCurrentSpeed());
                    Signal<Double> signal = new Signal<Double>(currentGap, timeStamp);
                    controllerOutput = speedController_.apply(signal).get().getData();
                }
            } else {
                double leaderCurrentPosition = leader.vehiclePosition;
                log_.info("The current leader position is " + leaderCurrentPosition);
                double hostVehiclePosition = pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance();
                double hostVehicleSpeed = pluginServiceLocator_.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
                log_.info("The host vehicle speed is + " + hostVehicleSpeed + " and its position is " + hostVehiclePosition);
                // If the host vehicle is the fifth vehicle and it is following the third vehicle, the leader index here is 2
                // vehiclesInFront should be 2, because number of vehicles in front is 4, then numOfVehiclesGaps = VehicleInFront - leaderIndex   
                int leaderIndex = plugin_.platoonManager.getIndexOf(leader);
                int numOfVehiclesGaps = plugin_.platoonManager.getNumberOfVehicleInFront() - leaderIndex;
                log_.info("The host vehicle have " + numOfVehiclesGaps + " vehicles between itself and its leader (includes the leader)");
                desiredGap_ = Math.max(hostVehicleSpeed * plugin_.timeHeadway * numOfVehiclesGaps, plugin_.standStillHeadway * numOfVehiclesGaps);
                log_.info("The desired gap with the leader is " + desiredGap_);
                log_.info("Based on raw radar, the current gap with the front vehicle is " + plugin_.getManeuverInputs().getDistanceToFrontVehicle());
                double desiredHostPosition = leaderCurrentPosition - this.desiredGap_;
                log_.info("The desired host position and the setpoint for pid controller is " + desiredHostPosition);
                // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
                // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
                // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
                // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
                // The command speed of leader vehicle will act as the baseline for our speed control
                distanceGapController_.changeSetpoint(desiredHostPosition);
                Signal<Double> signal = new Signal<Double>(hostVehiclePosition, timeStamp);
                controllerOutput = speedController_.apply(signal).get().getData();
            }
            double adjSpeedCmd = controllerOutput + leader.commandSpeed;
            log_.info("Adjusted Speed Cmd = " + adjSpeedCmd + "; Controller Output = " + controllerOutput
                     + "; Leader CmdSpeed= " + leader.commandSpeed + "; Adjustment Cap " + adjustmentCap);
            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
            if(enableMaxAdjustmentFilter) {
                if(adjSpeedCmd > leader.commandSpeed + this.adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed + this.adjustmentCap;
                } else if(adjSpeedCmd < leader.commandSpeed - this.adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed - this.adjustmentCap;
                }
                log_.info("The adjusted cmd speed after max adjustment cap is " + adjSpeedCmd + " m/s");
            }
            // Second: we do not exceed the local speed limit
            if(enableLocalSpeedLimitFilter) {
                SpeedLimit limit = pluginServiceLocator_.getRouteService().getSpeedLimitAtLocation(pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance());
                double localSpeedLimit = adjSpeedCmd;
                if(limit != null) {
                    localSpeedLimit = limit.getLimit();
                    log_.info("The local speed limit is " + localSpeedLimit + ", cap adjusted speed to speed limit if necessary");
                } else {
                    log_.warn("Cannot find local speed limit in current location" + pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance());
                }
                adjSpeedCmd = Math.min(Math.max(adjSpeedCmd, 0), localSpeedLimit);
                log_.info("The speed command after local limit cap is: " + adjSpeedCmd + " m/s");
            }
            // Third: we allow do not a large gap between two consecutive speed commands
            if(enableMaxAccelFilter) {
                if(!lastCmdSpeed.isPresent()) {
                    lastCmdSpeed = Optional.of(plugin_.getLastSpeedCmd());
                }
                double max = lastCmdSpeed.get() + (plugin_.maxAccel * (CMD_TIMESTEP / 1000.0));
                double min = lastCmdSpeed.get() - (plugin_.maxAccel * (CMD_TIMESTEP / 1000.0));
                if(adjSpeedCmd > max) {
                    adjSpeedCmd = max; 
                } else if (adjSpeedCmd < min) {
                    adjSpeedCmd = min;
                }
                lastCmdSpeed = Optional.of(adjSpeedCmd);
                log_.info("The speed command after max accel cap is: " + adjSpeedCmd + " m/s");
            }
            speedCmd_.set(adjSpeedCmd);
            log_.info("A speed command is generated from command generator: " + speedCmd_.get() + " m/s");
        } else {
            // TODO if there is no leader available, we should change back to Leader State and re-join other platoon later
            speedCmd_.set(plugin_.getManeuverInputs().getCurrentSpeed());
            distanceGapController_.reset();
        }
    }
    
}
