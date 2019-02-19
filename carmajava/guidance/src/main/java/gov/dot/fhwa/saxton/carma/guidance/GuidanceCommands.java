/*
* Copyright (C) 2018-2019 LEIDOS.
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
import cav_srvs.SetEnableRobotic;
import cav_srvs.SetEnableRoboticRequest;
import cav_srvs.SetEnableRoboticResponse;
import geometry_msgs.TwistStamped;

import com.google.common.util.concurrent.AtomicDouble;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import std_msgs.Float32;

import java.util.concurrent.atomic.AtomicBoolean;

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;

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
    private IPublisher<std_msgs.Float32> wrenchEffortPublisher;
    private ISubscriber<TwistStamped> velocitySubscriber;
    private AtomicDouble speedCommand = new AtomicDouble(-1.0); // -1 used to indicate unset
    private AtomicDouble maxAccel = new AtomicDouble(-1.0); // -1 used to indicate unset
    private AtomicDouble steeringCommand = new AtomicDouble();
    private AtomicDouble lateralAccel = new AtomicDouble(0.0);
    private AtomicDouble yawRate = new AtomicDouble(0.0);
    private long sleepDurationMillis = 100;
    private long lastTimestep = -1;
    private double vehicleAccelLimit = 2.5;
    private static final long CONTROLLER_TIMEOUT_PERIOD_MS = 200;
    public static final double MAX_SPEED_CMD_M_S = 35.7632; // 80 MPH, hardcoded to persist through configuration change 
    private final IManeuverInputs maneuverInputs;
    private AtomicBoolean usingWrenchEffort = new AtomicBoolean(false); // TODO remove if wrench effort override is removed
    private boolean useWrenchEffortStoppingOverride; // TODO remove if wrench effort override is removed

    GuidanceCommands(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node, IManeuverInputs maneuverInputs) {
        super(stateMachine, iPubSubService, node);
        this.maneuverInputs = maneuverInputs;
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
        useWrenchEffortStoppingOverride = node.getParameterTree().getBoolean("~use_wrench_effort_stopping_override", false);
        log.info("GuidanceCommands using max accel limit of " + vehicleAccelLimit);
        velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
        currentState.set(GuidanceState.STARTUP);
    }
    @Override
    public void onSystemReady() {

        speedAccelPublisher = pubSubService.getPublisherForTopic("cmd_speed", SpeedAccel._TYPE);
        wrenchEffortPublisher = pubSubService.getPublisherForTopic("cmd_longitudinal_effort", Float32._TYPE);
        try {
            enableRoboticService = pubSubService.getServiceForTopic("enable_robotic", SetEnableRobotic._TYPE);
        } catch (TopicNotFoundException tnfe) {
            exceptionHandler.handleException("GuidanceCommands unable to locate control/enable_robotic service", tnfe);
        }
        lateralControlPublisher = pubSubService.getPublisherForTopic("cmd_lateral", LateralControl._TYPE);
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onActive() {
        SetEnableRoboticRequest enableReq = enableRoboticService.newMessage();
        enableReq.setSet((byte) 1);

        // TODO: Implement no-response call method
        enableRoboticService.call(enableReq, new OnServiceResponseCallback<SetEnableRoboticResponse>() {
            @Override
            public void onSuccess(SetEnableRoboticResponse resp)  {
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
        enableRoboticService.call(enableReq, new OnServiceResponseCallback<SetEnableRoboticResponse>() {
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
        if(enableRoboticService != null) {
            enableRoboticService.close();
        }
    }

    @Override
    public void onPanic() {
        super.onPanic();
        if(enableRoboticService != null) {
            enableRoboticService.close();
        }
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
    public synchronized void setSpeedCommand(double speed, double accel) {
        if (speed > MAX_SPEED_CMD_M_S) {
            log.warn("GuidanceCommands attempted to set speed command (" + speed + " m/s) higher than maximum limit of "
                    + MAX_SPEED_CMD_M_S + " m/s. Capping to speed limit.");
            speed = MAX_SPEED_CMD_M_S;
        } else if (speed < 0.0) {
            log.warn("GuidanceCommands received negative command from maneuver, clamping to 0.0");
            speed = 0.0;
        }
        if (accel < 0.01) {
            log.warn("MaxAccel of ~0 proposed: " + accel);
        }

        speedCommand.set(speed);
        maxAccel.set(Math.min(Math.abs(accel), Math.abs(vehicleAccelLimit)));
        log.info("CONTROLS", "Speed command set to " + speedCommand.get() + "m/s and " + maxAccel.get() + "m/s/s");
    }

    @Override
    public synchronized void setSteeringCommand(double axleAngle, double lateralAccel, double yawRate) {
        axleAngle = Math.max(axleAngle, -Math.PI / 2.0);
        axleAngle = Math.min(axleAngle, Math.PI / 2.0);

        steeringCommand.set(axleAngle);
        this.lateralAccel.set(lateralAccel);
        this.yawRate.set(yawRate);

        log.info("CONTROLS", "Steering command set to " + axleAngle + " rad axle angle," + lateralAccel
                + " m/s/s lateral accel, and " + yawRate + " rad/s yaw rate.");
    }

    @Override
    public void timingLoop() throws InterruptedException {
        // TODO This method should be synchronized. Remove it to test its impact on timing
        // Iterate ensuring smooth speed command output
        long iterStartTime = System.currentTimeMillis();

        if (currentState.get() == GuidanceState.ENGAGED) {
                SpeedAccel msg = speedAccelPublisher.newMessage();
                
                double cachedSpeed = 0;
                double cachedMaxAccel = 0;
                synchronized(this) {
                    cachedSpeed = speedCommand.get();
                    cachedMaxAccel = maxAccel.get();
                }
    
                // TODO This is a special case fix for the 2013 Cadillac SRX TORC speed controller
                // If the vehicle wants to stand still (0 mph) we will command with wrench effort instead
                // This should be refactored or removed once the STOL TO 26 demo is complete
                // TODO This logic should be refactored into the Cadillac controller driver which should handle the bug internally. 
                final double SIX_MPH = 2.68224;
                if (useWrenchEffortStoppingOverride
                    && Math.abs(cachedSpeed) < 0.1
                    && Math.abs(cachedMaxAccel) - maneuverInputs.getMaxAccelLimit() < 0.00001
                    && maneuverInputs.getCurrentSpeed() < SIX_MPH) {
                    std_msgs.Float32 effortMsg = wrenchEffortPublisher.newMessage();
                    effortMsg.setData(-100.0f);
                    wrenchEffortPublisher.publish(effortMsg);
                    usingWrenchEffort.set(true);
                } else {
                    msg.setSpeed(speedCommand.get());
                    msg.setMaxAccel(maxAccel.get());
                    speedAccelPublisher.publish(msg);
                }
                
                cav_msgs.LateralControl lateralMsg = lateralControlPublisher.newMessage();
                synchronized(this) {
                    lateralMsg.setAxleAngle(steeringCommand.get());
                    lateralMsg.setMaxAccel(lateralAccel.get());
                    lateralMsg.setMaxAxleAngleRate(yawRate.get());
                }
                lateralControlPublisher.publish(lateralMsg);
                log.trace("Published longitudinal & lateral cmd message after "
                        + (System.currentTimeMillis() - iterStartTime) + "ms.");
        } else if (currentState.get() == GuidanceState.ACTIVE || currentState.get() == GuidanceState.INACTIVE) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            double current_speed = 0.0;
            if (velocitySubscriber.getLastMessage() != null) {
                current_speed = velocitySubscriber.getLastMessage().getTwist().getLinear().getX();
                
                current_speed = Math.min(Math.max(current_speed, 0.0), MAX_SPEED_CMD_M_S);
            }
            // Set the speed/accel commands to ensure valid values are passed to the controller before we engage
            // Comparing against default value of -1 to avoid race condition where first maneuver starts executing before this loop is called after engagement
            speedCommand.compareAndSet(-1.0, current_speed);
            maxAccel.compareAndSet(-1.0, maneuverInputs.getMaxAccelLimit());

            msg.setSpeed(speedCommand.get());
            //TODO maybe need to change maxAccel and commands in lateralMsgs
            msg.setMaxAccel(maxAccel.get());
            speedAccelPublisher.publish(msg);

            cav_msgs.LateralControl lateralMsg = lateralControlPublisher.newMessage();
            lateralMsg.setAxleAngle(0.0);
            lateralMsg.setMaxAccel(0.0);
            lateralMsg.setMaxAxleAngleRate(0.0);
            lateralControlPublisher.publish(lateralMsg);
            log.trace("Published longitudinal & lateral cmd message after "
                    + (System.currentTimeMillis() - iterStartTime) + "ms.");
        }

        long iterEndTime = System.currentTimeMillis();

        // Not our first timestep, check timestep spacings
        if (currentState.get() == GuidanceState.ENGAGED && lastTimestep > -1) {
            if (iterEndTime - lastTimestep > CONTROLLER_TIMEOUT_PERIOD_MS) {
                log.error("!!!!! GUIDANCE COMMANDS LOOP EXCEEDED CONTROLLER TIMEOUT AFTER "
                        + (iterEndTime - lastTimestep) + "ms. CONTROLLER MAY BE UNRESPONSIVE. !!!!!");
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
            jobQueue.add(this::onActive);
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
            throw new RosRuntimeException(
                    getComponentName() + "received unknown instruction from guidance state machine.");
        }
    }
}
