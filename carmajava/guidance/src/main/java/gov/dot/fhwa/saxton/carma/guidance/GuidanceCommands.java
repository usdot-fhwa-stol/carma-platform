/*
* Copyright (C) 2017 LEIDOS.
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

import cav_msgs.SpeedAccel;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import cav_srvs.SetEnableRobotic;
import cav_srvs.SetEnableRoboticRequest;
import cav_srvs.SetEnableRoboticResponse;
import gov.dot.fhwa.saxton.carma.rosutils.*;

import com.google.common.util.concurrent.AtomicDouble;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.*;

/**
* GuidanceCommands is the guidance sub-component responsible for maintaining consistent control of the vehicle.
*
* GuidanceCommands' primary function is to ensure that controller timeouts do not occur during normal
* operation of the CARMA platform. It does so by buffering commands received from the TrajectoryExecutor
* and it's Maneuver instances and latching on those commands until a new one is received. This will output
* the most recently latched value at a fixed frequency.
*/
public class GuidanceCommands extends GuidanceComponent implements IGuidanceCommands {
    private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    private IPublisher<SpeedAccel> speedAccelPublisher;
    private IService<SetEnableRoboticRequest, SetEnableRoboticResponse> enableRoboticService;
    private AtomicDouble speedCommand = new AtomicDouble(0.0);
    private AtomicDouble maxAccel = new AtomicDouble(0.0);
    private long sleepDurationMillis = 100;
    private long lastTimestep = -1;
    private AtomicBoolean engaged = new AtomicBoolean(false);
    private boolean driverConnected = false;
    private double vehicleAccelLimit = 2.5;
    private static final String SPEED_CMD_CAPABILITY = "control/cmd_speed";
    private static final String ENABLE_ROBOTIC_CAPABILITY = "control/enable_robotic";
    private static final long CONTROLLER_TIMEOUT_PERIOD_MS = 200;
    public static final double MAX_SPEED_CMD_M_S = 35.7632; // 80 MPH, hardcoded to persist through configuration change 

    GuidanceCommands(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
        super(state, iPubSubService, node);
    }

    @Override
    public String getComponentName() {
        return "Guidance.Commands";
    }

    @Override
    public void onGuidanceStartup() {
        vehicleAccelLimit = node.getParameterTree().getDouble("~vehicle_acceleration_limit", 2.5);
        log.info("GuidanceCommands using max accel limit of " + vehicleAccelLimit);
    }

    @Override
    public void onGuidanceEnable() {
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
                // NO-OP
            }
        });

        engaged.set(true);
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
    public void setCommand(double speed, double accel) {
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
    public void onSystemReady() {
        // Register with the interface manager's service
        try {
            driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                    GetDriversWithCapabilities._TYPE);
        } catch (TopicNotFoundException tnfe) {
            panic("Interface manager not found.");
        }

        // Build our request message
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
                log.info("Received GetDriversWithCapabilitiesResponse");
                for (String driverName : msg.getDriverData()) {
                    log.info("GuidanceCommands discovered driver: " + driverName);
                }

                drivers[0] = msg;
            }

            @Override
            public void onFailure(Exception e) {
                panic("InterfaceManager failed to return a control/cmd_speed capable driver!!!");
            }
        });

        // No message for LanePosition.msg to be published on "guidance/control/lane_position"
        // TODO: Add message type for lateral control from guidance

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
                driverConnected = true;
            } catch (TopicNotFoundException tnfe) {
                panic("GuidanceCommands unable to locate control/enable_robotic service for " + roboticEnableTopic);
            }
        } else {
            panic("GuidanceCommands unable to find suitable controller driver!");
        }
    }

    @Override
    public void loop() throws InterruptedException {
        // Iterate ensuring smooth speed command output
        long iterStartTime = System.currentTimeMillis();

        if (engaged.get() && driverConnected) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            msg.setSpeed(speedCommand.get());
            msg.setMaxAccel(maxAccel.get());
            speedAccelPublisher.publish(msg);
            log.trace("Published cmd message after " + (System.currentTimeMillis() - iterStartTime) + "ms.");
        }

        long iterEndTime = System.currentTimeMillis();

        // Not our first timestep, check timestep spacings
        if (engaged.get() && driverConnected && lastTimestep > -1) {
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
}
