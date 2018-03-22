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

import com.google.common.util.concurrent.AtomicDouble;

import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.signals.Filter;
import gov.dot.fhwa.saxton.carma.guidance.signals.PidController;
import gov.dot.fhwa.saxton.carma.guidance.signals.Pipeline;
import gov.dot.fhwa.saxton.carma.guidance.signals.Signal;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;

/**
 * This class generates speed commands based on the latest information from plugin platoon list.
 */
public class CommandGenerator implements Runnable, IPlatooningCommandInputs {
    
    protected PlatooningPlugin plugin_;
    protected PluginServiceLocator pluginServiceLocator_;
    protected ILogger log_;
    protected long timestep_;
    protected Filter<Double> distanceGapController_;
    protected Pipeline<Double> speedController_;
    protected AtomicDouble speedCmd_ = new AtomicDouble(0.0);
    
    public CommandGenerator(PlatooningPlugin plugin, ILogger log, PluginServiceLocator pluginServiceLocator) {
        this.plugin_ = plugin;
        this.pluginServiceLocator_ = pluginServiceLocator;
        this.log_ = log;
        this.distanceGapController_ = new PidController(plugin_.getKpPID(), plugin_.getKiPID(), plugin_.getKdPID(), plugin_.getStandStillGap());
        this.speedController_ = new Pipeline<Double>(distanceGapController_);
    }

    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                long tsStart = System.currentTimeMillis();
                generateSpeed((double) tsStart);
                long tsEnd = System.currentTimeMillis();
                long sleepDuration = Math.max(timestep_ - (tsEnd - tsStart), 0);
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
        return plugin_.getMaxAccel();
    }
    
    protected void generateSpeed(double timeStamp) {
        // Update speed commands based on the list of platoon members
        PlatoonMember leader = null;
        if(plugin_.getPlatoonManager() != null) {
            leader = plugin_.getPlatoonManager().getLeader();
        }
        if(leader != null) {
            double leaderCurrentPosition = leader.getVehiclePosition();
            log_.debug("The current leader position is " + leaderCurrentPosition);
            double hostVehiclePosition = pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance();
            double hostVehicleSpeed = pluginServiceLocator_.getManeuverPlanner().getManeuverInputs().getCurrentSpeed();
            log_.debug("The host vehicle speed is + " + hostVehicleSpeed + " and its position is " + hostVehiclePosition);
            int vehiclesInFront = plugin_.getPlatoonManager().getPlatooningSize();
            log_.debug("The host vehicle have " + vehiclesInFront + " vehicles in front of it");
            double desiredLeaderPosition = hostVehiclePosition + Math.max(hostVehicleSpeed * plugin_.getTimeHeadway() * vehiclesInFront, plugin_.getStandStillGap());
            log_.debug("The desired leader position and the setpoint for pid controller is " + desiredLeaderPosition);
            // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
            // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
            // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
            // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
            // The command speed of leader vehicle will act as the baseline for our speed control
            distanceGapController_.changeSetpoint(desiredLeaderPosition);
            Signal<Double> signal = new Signal<Double>(leaderCurrentPosition, timeStamp);
            double output = speedController_.apply(signal).get().getData();
            log_.debug("The output from controller is " + output);
            speedCmd_.set(Math.max(output + leader.getCommandSpeed(), 0));
            log_.debug("A speed command is generated from pid controller: " + speedCmd_ + " m/s");
        } else {
            log_.warn("CommandGenerator can not find the leader, starting latching speed commands if complex maneuver is still running");
            // TODO if there is no leader avaliable, we should change back to Leader State and rejoin other platoon later
            speedCmd_.set(pluginServiceLocator_.getManeuverPlanner().getManeuverInputs().getCurrentSpeed());
            distanceGapController_.reset();
        }
    }
    
}
