package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.SpeedAccel;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import std_msgs.Float32;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * GuidanceCommands is the guidance sub-component responsible for maintaining consistent control of the vehicle.
 * <p>
 * GuidanceCommands' primary function is to ensure that controller timeouts do not occur during normal
 * operation of the CARMA platform. It does so by buffering commands received from the TrajectoryExecutor
 * and it's Maneuver instances and latching on those commands until a new one is received. This will output
 * the most recently latched value at a fixed frequency.
 * <p>
 * Presently this is just a mock to implement messaging functionality. during normal
 */
public class GuidanceCommands extends GuidanceComponent {
    private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    private IPublisher<SpeedAccel> speedAccelPublisher;

    GuidanceCommands(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
        super(state, iPubSubService, node);
    }

    @Override
    public String getComponentName() {
        return "Guidance.Commands";
    }

    @Override
    public void onGuidanceStartup() {

    }

    @Override
    public void onGuidanceEnable() {

    }

    @Override
    public void onSystemReady() {
        try {
            driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                    GetDriversWithCapabilities._TYPE);

            GetDriversWithCapabilitiesRequest req = node.getServiceRequestMessageFactory()
                    .newFromType(GetDriversWithCapabilitiesRequest._TYPE);

            List<String> reqdCapabilities = new ArrayList<>();
            reqdCapabilities.add("control/cmd_speed");
            req.setCapabilities(reqdCapabilities);
            final GetDriversWithCapabilitiesResponse[] drivers = new GetDriversWithCapabilitiesResponse[1];
            drivers[0] = null;
            driverCapabilityService.call(req, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
                @Override
                public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                    log.info("Received GetDriversWithCapabilitiesResponse:" + msg);
                    drivers[0] = msg;
                }

                @Override
                public void onFailure(Exception e) {
                    log.warn("No control/cmd_speed capable driver found!!!");
                }
            });

            // No message for LanePosition.msg to be published on "guidance/control/lane_position"
            // TODO: Add message type for lateral control from guidance

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
                speedAccelPublisher = pubSubService.getPublisherForTopic(driverFqn, SpeedAccel._TYPE);
            }
        } catch (TopicNotFoundException e) {
            log.error("No interface manager found to query for drivers!!!");
            e.printStackTrace();
        }
    }
}
