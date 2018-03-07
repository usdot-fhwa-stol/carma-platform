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
import geometry_msgs.TwistStamped;

import com.google.common.util.concurrent.AtomicDouble;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import java.util.ArrayList;
import java.util.List;
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
    private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    private IPublisher<SpeedAccel> speedAccelPublisher;
    private IService<SetEnableRoboticRequest, SetEnableRoboticResponse> enableRoboticService;
    private IPublisher<cav_msgs.LateralControl> lateralControlPublisher;
    private IService<SetLightsRequest, SetLightsResponse> setLightsService;
    private ISubscriber<TwistStamped> velocitySubscriber;
    private AtomicDouble speedCommand = new AtomicDouble(0.0);
    private AtomicDouble maxAccel = new AtomicDouble(0.0);
    private AtomicDouble steeringCommand = new AtomicDouble(0.0);
    private AtomicDouble lateralAccel = new AtomicDouble(0.0);
    private AtomicDouble yawRate = new AtomicDouble(0.0);
    private long sleepDurationMillis = 100;
    private long lastTimestep = -1;
    private double vehicleAccelLimit = 2.5;
    private static final String SPEED_CMD_CAPABILITY = "control/cmd_speed";
    private static final String ENABLE_ROBOTIC_CAPABILITY = "control/enable_robotic";
    private static final String LATERAL_CONTROL_CAPABILITY = "control/cmd_lateral";
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
        velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
        currentState.set(GuidanceState.STARTUP);
    }
    @Override
    public void onSystemReady() {
        // Register with the interface manager's service
        try {
            driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
        } catch (TopicNotFoundException tnfe) {
            exceptionHandler.handleException("Interface manager not found.", tnfe);
        }

        // Build our request message for longitudinal control drivers
        GetDriversWithCapabilitiesRequest req = driverCapabilityService.newMessage();

        List<String> reqdCapabilities = new ArrayList<>();
        reqdCapabilities.add(SPEED_CMD_CAPABILITY);
        reqdCapabilities.add(ENABLE_ROBOTIC_CAPABILITY);
        req.setCapabilities(reqdCapabilities);

        // Work around to pass a final object into our anonymous inner class so we can get the
        // response
        final GetDriversWithCapabilitiesResponse[] drivers = new GetDriversWithCapabilitiesResponse[1];
        drivers[0] = null;

        // Call the InterfaceManager to see if we have a driver that matches our requirements
        driverCapabilityService.callSync(req, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
            @Override
            public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                log.debug("Received GetDriversWithCapabilitiesResponse");
                for (String driverName : msg.getDriverData()) {
                    log.debug("GuidanceCommands discovered driver: " + driverName);
                }

                drivers[0] = msg;
            }

            @Override
            public void onFailure(Exception e) {
                exceptionHandler.handleException("InterfaceManager failed to return a control/cmd_speed capable driver!!!", e);
            }
        });

        // Verify that the message returned drivers that we can use
        String speedCmdTopic = null;
        String roboticEnableTopic = null;
        if (drivers[0] != null) {
            for (String topicName : drivers[0].getDriverData()) {
                if (topicName.endsWith(SPEED_CMD_CAPABILITY)) {
                    speedCmdTopic = topicName;
                }
                if (topicName.endsWith(ENABLE_ROBOTIC_CAPABILITY)) {
                    roboticEnableTopic = topicName;
                }
            }
        }

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

        // Repeat the above process for lateral control drivers
        GetDriversWithCapabilitiesRequest lateralReq = driverCapabilityService.newMessage();
        List<String> lateralCapabilities = new ArrayList<>();
        lateralCapabilities.add(LATERAL_CONTROL_CAPABILITY);
        lateralReq.setCapabilities(lateralCapabilities);
        final GetDriversWithCapabilitiesResponse[] lateralDrivers = new GetDriversWithCapabilitiesResponse[1];
        lateralDrivers[0] = null;
        driverCapabilityService.callSync(lateralReq, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
            @Override
            public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                log.debug("Received GetDriversWithCapabilitiesResponse");
                for (String driverName : msg.getDriverData()) {
                    log.debug("GuidanceCommands discovered driver: " + driverName);
                }

                lateralDrivers[0] = msg;
            }

            @Override
            public void onFailure(Exception e) {
                exceptionHandler.handleException("InterfaceManager failed to return a control/cmd_lateral capable driver!!!", e);
            }
        });
        String lateralControlTopic = null;
        if (lateralDrivers[0] != null) {
            for (String topicName : lateralDrivers[0].getDriverData()) {
                if (topicName.endsWith(LATERAL_CONTROL_CAPABILITY)) {
                    lateralControlTopic = topicName;
                }
            }
        }
        if (lateralControlTopic == null) {
            exceptionHandler.handleException("GuidanceCommands unable to find suitable lateral controller driver!", new RosRuntimeException("No lateral controller drivers."));
        }
        
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onActive() {
        SetEnableRoboticRequest enableReq = enableRoboticService.newMessage();
        enableReq.setSet((byte) 1);

        // TODO: Implement no-response call method
        enableRoboticService.callSync(enableReq, new OnServiceResponseCallback<SetEnableRoboticResponse>() {
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
    public synchronized void setSpeedCommand(double speed, double accel) {
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
        // Iterate ensuring smooth speed command output
        long iterStartTime = System.currentTimeMillis();

        if (currentState.get() == GuidanceState.ENGAGED) {
            synchronized (this) {
                SpeedAccel msg = speedAccelPublisher.newMessage();
                msg.setSpeed(speedCommand.get());
                msg.setMaxAccel(maxAccel.get());
                speedAccelPublisher.publish(msg);

                cav_msgs.LateralControl lateralMsg = lateralControlPublisher.newMessage();
                lateralMsg.setAxleAngle(steeringCommand.get());
                lateralMsg.setMaxAccel(lateralAccel.get());
                lateralMsg.setMaxAxleAngleRate(yawRate.get());
                lateralControlPublisher.publish(lateralMsg);
                log.trace("Published longitudinal & lateral cmd message after "
                        + (System.currentTimeMillis() - iterStartTime) + "ms.");
            }
        } else if (currentState.get() == GuidanceState.ACTIVE || currentState.get() == GuidanceState.INACTIVE) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            double current_speed = 0.0;
            if (velocitySubscriber.getLastMessage() != null) {
                current_speed = velocitySubscriber.getLastMessage().getTwist().getLinear().getX();
                if (current_speed < 0) {
                    current_speed = 0.0;
                } else {
                    current_speed = Math.min(current_speed, MAX_SPEED_CMD_M_S);
                }
            }
            msg.setSpeed(current_speed);
            //TODO maybe need to change maxAccel and commands in lateralMsgs
            msg.setMaxAccel(1.0);
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
                    getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
}
