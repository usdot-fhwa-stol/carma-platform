package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.SpeedAccel;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import com.google.common.util.concurrent.AtomicDouble;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.apache.commons.logging.Log;
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
 *
 * Presently this is just a mock to implement messaging functionality. during normal
 */
public class GuidanceCommands extends GuidanceComponent {
    private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    private IPublisher<SpeedAccel> speedAccelPublisher;
    private AtomicDouble speedCommand = new AtomicDouble(0.0);
    private AtomicDouble maxAccel = new AtomicDouble(0.0);
    private long sleepDurationMillis = 100;
    private AtomicBoolean engaged = new AtomicBoolean(false);

    GuidanceCommands(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
        super(state, iPubSubService, node);
    }

    @Override public String getComponentName() {
        return "Guidance.Commands";
    }

    @Override public void onGuidanceStartup() {

    }

    @Override public void onGuidanceEnable() {
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
    public void setCommand(double speed, double accel) {
        speedCommand.set(speed);
        maxAccel.set(accel);
        log.info("Speed command set to "  + speed + "m/s and " + accel + "m/s/s");
    }

    @Override public void onSystemReady() {
            // Register with the interface manager's service
            try {
                driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                    GetDriversWithCapabilities._TYPE);
            } catch (Exception tnfe) {
                log.fatal("Interface manager not found. Shutting down Guidance.Commands");
            }

            // Build our request message
            GetDriversWithCapabilitiesRequest req =
                node.getServiceRequestMessageFactory()
                    .newFromType(GetDriversWithCapabilitiesRequest._TYPE);

            List<String> reqdCapabilities = new ArrayList<>();
            reqdCapabilities.add("cmd_speed"); // We only need to use one type of control
            req.setCapabilities(reqdCapabilities);

            // Work around to pass a final object into our anonymous inner class so we can get the
            // response
            final GetDriversWithCapabilitiesResponse[] drivers =
                new GetDriversWithCapabilitiesResponse[1];
            drivers[0] = null;

            // Call the InterfaceManager to see if we have a driver that matches our requirements
            driverCapabilityService.call(req,
                new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
                    @Override public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                        log.info("Received GetDriversWithCapabilitiesResponse:" + msg);
                        drivers[0] = msg;
                    }

                    @Override public void onFailure(Exception e) {
                        log.warn("No control/cmd_speed capable driver found!!!");
                    }
                });

            // No message for LanePosition.msg to be published on "guidance/control/lane_position"
            // TODO: Add message type for lateral control from guidance

            // Verify that the message returned drivers that we can use
            String driverFqn = null;
            if (drivers[0] != null) {
                List<String> driverFqns = drivers[0].getDriverData();
                if (driverFqns.size() > 0) {
                    driverFqn = driverFqns.get(0);
                    log.info("Discovered driver: " + driverFqns.get(0));
                } else {
                    log.warn("No control/cmd_speed capable driver found!!!");
                }
            } else {
                log.warn("No control/cmd_speed capable driver found!!!");
            }

            if (driverFqn != null) {
                // Open the publication channel to the driver and start sending it commands
                speedAccelPublisher = pubSubService.getPublisherForTopic(driverFqn, SpeedAccel._TYPE);

            } else {
                log.fatal("GuidanceCommands UNABLE TO FIND CONTROLLER DRIVER!");
            }
    }

    @Override
    public void loop() {
        // Iterate ensuring smooth speed command output
        long iterStartTime = System.currentTimeMillis();

        if (engaged.get()) {
            SpeedAccel msg = speedAccelPublisher.newMessage();
            msg.setSpeed(speedCommand.get());
            msg.setMaxAccel(maxAccel.get());
            speedAccelPublisher.publish(msg);
        }

        long iterEndTime = System.currentTimeMillis();

        try {
            Thread.sleep(sleepDurationMillis - (iterEndTime - iterStartTime));
        } catch (InterruptedException ie) {
            log.warn("Guidance.Commands interrupted... Shutting down.");
            Thread.currentThread().interrupt();
        }
    }
}
