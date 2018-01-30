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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.LateralControl;
import cav_msgs.SpeedAccel;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import cav_srvs.SetEnableRobotic;
import cav_srvs.SetEnableRoboticRequest;
import cav_srvs.SetEnableRoboticResponse;
import cav_srvs.SetLights;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;

import com.google.common.util.concurrent.AtomicDouble;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;

/**
* GuidanceCommands is the guidance sub-component responsible for maintaining consistent control of the vehicle.
*
* GuidanceCommands' primary function is to ensure that controller timeouts do not occur during normal
* operation of the CARMA platform. It does so by buffering commands received from the TrajectoryExecutor
* and it's Maneuver instances and latching on those commands until a new one is received. This will output
* the most recently latched value at a fixed frequency.
*/
public class GuidanceCommands extends GuidanceComponent implements IGuidanceCommands, IStateChangeListener {
    private IPublisher<SpeedAccel> speedAccelPublisher;
    private IService<SetEnableRoboticRequest, SetEnableRoboticResponse> enableRoboticService;
    private IPublisher<cav_msgs.LateralControl> lateralControlPublisher;
    private IService<SetLightsRequest, SetLightsResponse> setLightsService;
    private AtomicDouble speedCommand = new AtomicDouble(0.0);
    private AtomicDouble maxAccel = new AtomicDouble(0.0);
    private AtomicDouble steeringCommand = new AtomicDouble(0.0);
    private AtomicDouble lateralAccel = new AtomicDouble(0.0);
    private AtomicDouble yawRate = new AtomicDouble(0.0);
    private long sleepDurationMillis = 100;
    private long lastTimestep = -1;
    private double vehicleAccelLimit = 2.5;
    private static final String DRIVER_BASE_PATH = "/saxton_cav/drivers";
    private static final String SRX_CONTROLLER_PATH = "/srx_controller/";
    private static final String LATERAL_CONTROLLER_PATH = "/lateral_controller/";
    private static final String SPEED_CMD_CAPABILITY = "control/cmd_speed";
    private static final String ENABLE_ROBOTIC_CAPABILITY = "control/enable_robotic";
    private static final String LATERAL_CONTROL_CAPABILITY =  "control/cmd_lateral";
    private static final String LIGHT_CONTROL_CAPABILITY =  "control/set_lights";
    private static final long CONTROLLER_TIMEOUT_PERIOD_MS = 200;
    public static final double MAX_SPEED_CMD_M_S = 35.7632; // 80 MPH, hardcoded to persist through configuration change 

    GuidanceCommands(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node) {
        super(stateMachine, iPubSubService, node);
        this.jobQueue.add(this::onStartup);
        stateMachine.registerStateChangeListener(this);
    }

    @Override
    public String getComponentName() {
        return "Guidance.Commands";
    }

    @Override
    public void onStartup() {
            vehicleAccelLimit = node.getParameterTree().getDouble("~vehicle_acceleration_limit", 2.5);
            log.info("GuidanceCommands using max accel limit of " + vehicleAccelLimit);
            currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        String speedCmdTopic = DRIVER_BASE_PATH + SRX_CONTROLLER_PATH + SPEED_CMD_CAPABILITY;
        String roboticEnableTopic = DRIVER_BASE_PATH + SRX_CONTROLLER_PATH + ENABLE_ROBOTIC_CAPABILITY;

        if (speedCmdTopic != null && roboticEnableTopic != null) {
            // Open the publication channel to the driver and start sending it commands
            log.info("CONTROLS", "GuidanceCommands connecting to " + speedCmdTopic + " and " + roboticEnableTopic);

            speedAccelPublisher = pubSubService.getPublisherForTopic(speedCmdTopic, SpeedAccel._TYPE);

            try {
                enableRoboticService = pubSubService.getServiceForTopic(roboticEnableTopic, SetEnableRobotic._TYPE);
            } catch (TopicNotFoundException tnfe) {
                exceptionHandler.handleException("GuidanceCommands unable to locate control/enable_robotic service", tnfe);
            }
        } else {
            exceptionHandler.handleException("GuidanceCommands unable to find suitable longitudinal controller driver!", new RosRuntimeException("No longitudinal controller drivers."));
        }

        String lateralControlTopic = DRIVER_BASE_PATH + LATERAL_CONTROLLER_PATH + LATERAL_CONTROL_CAPABILITY;
        if (lateralControlTopic == null) { exceptionHandler.handleException("GuidanceCommands unable to find suitable lateral controller driver!", new RosRuntimeException("No lateral controller drivers."));
        }
        lateralControlPublisher = pubSubService.getPublisherForTopic(lateralControlTopic, LateralControl._TYPE);
        
        currentState.set(GuidanceState.DRIVERS_READY);
    }
    
    @Override
    public void onRouteActive() {
        SetEnableRoboticRequest enableReq = enableRoboticService.newMessage();
        enableReq.setSet((byte) 1);

        // TODO: Implement no-response call method
        enableRoboticService.callSync(enableReq, new OnServiceResponseCallback<SetEnableRoboticResponse>() {
            @Override
            public void onSuccess(SetEnableRoboticResponse resp) {
                // NO-OP
            }

            @Override
            public void onFailure(Exception e) {
                exceptionHandler.handleException("Unable to call enable robotic service", e);
            }
        });
        currentState.set(GuidanceState.ACTIVE);
    }
    
    @Override
    public void onDeactivate() {
        currentState.set(GuidanceState.INACTIVE);
    }
    
    @Override
    public void onEngaged() {
        currentState.set(GuidanceState.ENGAGED);
    }

    @Override
    public void onCleanRestart() {
        currentState.set(GuidanceState.DRIVERS_READY);
        
        //Reset member variables
        speedCommand.set(0.0);
        maxAccel.set(0.0);
        lastTimestep = -1;
        
        SetEnableRoboticRequest enableReq = enableRoboticService.newMessage();
        enableReq.setSet((byte) 0);

        // TODO: Implement no-response call method
        enableRoboticService.callSync(enableReq, new OnServiceResponseCallback<SetEnableRoboticResponse>() {
            @Override
            public void onSuccess(SetEnableRoboticResponse resp) {
                // NO-OP
            }

            @Override
            public void onFailure(Exception e) {
                exceptionHandler.handleException("Unable to call disable robotic service", e);
            }
        });
    }

    @Override
    public void onShutdown() {
        super.onShutdown();
        enableRoboticService.close();
        setLightsService.close();
    }

    @Override
    public void onPanic() {
        super.onPanic();
        enableRoboticService.close();
        setLightsService.close();
    }
    
    /**
    * Change the current output of the GuidanceCommands thread.
    *
    * GuidanceCommands will output the specified speed and accel commands at the configured
    * frequency until new values are set. This function is thread safe through usage of
    * {@link AtomicDouble} functionality
    *
    * @param speed The speed to output
    * @param accel The maximum allowable acceleration in attaining and maintaining that speed
    */
    @Override
    public void setSpeedCommand(double speed, double accel) {
        if (speed > MAX_SPEED_CMD_M_S) {
            log.warn("GuidanceCommands attempted to set speed command (" + speed + " m/s) higher than maximum limit of "
                    + MAX_SPEED_CMD_M_S + " m/s. Capping to speed limit.");
            speed = MAX_SPEED_CMD_M_S;
        } else if (speed < 0.0) {
            log.warn("GuidanceCommands received negative command from maneuver, clamping to 0.0");
            speed = 0.0;
        }

        speedCommand.set(speed);
        maxAccel.set(Math.min(Math.abs(accel), Math.abs(vehicleAccelLimit)));
        log.info("CONTROLS", "Speed command set to " + speedCommand.get() + "m/s and " + maxAccel.get() + "m/s/s");
    }

    @Override
    public void setSteeringCommand(double axleAngle, double lateralAccel, double yawRate) {
        axleAngle = Math.max(axleAngle, -Math.PI/2.0);
        axleAngle = Math.min(axleAngle, Math.PI/2.0);

        steeringCommand.set(axleAngle);
        this.lateralAccel.set(lateralAccel);
        this.yawRate.set(yawRate);

        log.info("CONTROLS", "Steering command set to " + axleAngle + " rad axle angle," 
        + lateralAccel + " m/s/s lateral accel, and " 
        + yawRate + " rad/s yaw rate.");
    }
    
    @Override
    public void timingLoop() throws InterruptedException {
        // Iterate ensuring smooth speed command output
        long iterStartTime = System.currentTimeMillis();

        if (currentState.get() == GuidanceState.ENGAGED) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            msg.setSpeed(speedCommand.get());
            msg.setMaxAccel(maxAccel.get());
            speedAccelPublisher.publish(msg);

            cav_msgs.LateralControl lateralMsg = lateralControlPublisher.newMessage();
            lateralMsg.setAxleAngle(steeringCommand.get());
            lateralMsg.setMaxAccel(lateralAccel.get());
            lateralMsg.setMaxAxleAngleRate(yawRate.get());
            lateralControlPublisher.publish(lateralMsg);
            log.trace("Published longitudinal & lateral cmd message after " + (System.currentTimeMillis() - iterStartTime) + "ms.");
        } else if (currentState.get() == GuidanceState.ACTIVE) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            msg.setSpeed(0.0);
            msg.setMaxAccel(1.0);
            speedAccelPublisher.publish(msg);

            cav_msgs.LateralControl lateralMsg = lateralControlPublisher.newMessage();
            lateralMsg.setAxleAngle(0.0);
            lateralMsg.setMaxAccel(0.0);
            lateralMsg.setMaxAxleAngleRate(0.0);
            lateralControlPublisher.publish(lateralMsg);
            log.trace("Published longitudinal & lateral cmd message after " + (System.currentTimeMillis() - iterStartTime) + "ms.");
        }

        long iterEndTime = System.currentTimeMillis();

        // Not our first timestep, check timestep spacings
        if (currentState.get() == GuidanceState.ENGAGED && lastTimestep > -1) {
            if (iterEndTime - lastTimestep > CONTROLLER_TIMEOUT_PERIOD_MS) {
                log.error(
                        "!!!!! GUIDANCE COMMANDS LOOP EXCEEDED CONTROLLER TIMEOUT AFTER " 
                        + (iterEndTime - lastTimestep) 
                        + "ms. CONTROLLER MAY BE UNRESPONSIVE. !!!!!");
            }
        }

        lastTimestep = iterEndTime;

        Thread.sleep(Math.max(sleepDurationMillis - (iterEndTime - iterStartTime), 0));
    }

    /*
     * This method add the right job in the jobQueue base on the instruction given by GuidanceStateMachine
     * The actual changing of GuidanceState local copy is happened when each job is performed
     */
    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onRouteActive);
            break;
        case DEACTIVATE:
            jobQueue.add(this::onDeactivate);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
        case PANIC_SHUTDOWN:
            jobQueue.add(this::onPanic);
            break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
}
