/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package TrajectoryExecutor component
 * <p>
 * Guidance component responsible for performing the timely execution of planned
 * maneuvers in a Trajectory planned by the Arbitrator and the Guidance package's
 * currently configured plugins.
 */
public class TrajectoryExecutor extends GuidanceComponent {
    // Member variables
<<<<<<< 05a81d40e975ce23715891e36b8f9b95273d5f96
    private ISubscriber<RouteState> routeStateSubscriber;

    public TrajectoryExecutor(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
        super(state, pubSubService, node);
=======
    protected final String componentName = "TrajectoryExecutor";
    protected final long sleepDurationMillis = 30000;
    protected IPubSubService iPubSubService;
    protected int sequenceNumber = 0;
    protected Log log;
    protected ConnectedNode node;
    protected GuidanceCommands commands;
    protected double amplitude;
    protected double phase;
    protected double frequency;
    protected double maxAccel = 10.0;

    public TrajectoryExecutor(IPubSubService iPubSubService, GuidanceCommands commands,ConnectedNode node) {
        this.iPubSubService = iPubSubService;
        this.node= node;
        this.log = node.getLog();
        this.commands = commands;
>>>>>>> Initial implementation of GuidanceCommands functionality
    }

    @Override public String getComponentName() {
        return "Guidance.TrajectoryExecutor";
    }

    @Override public void onGuidanceStartup() {
        routeStateSubscriber = pubSubService
            .getSubscriberForTopic("route_status", RouteState._TYPE);
        routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override public void onMessage(RouteState msg) {
                log.info("Received RouteState:" + msg);
            }
        });

<<<<<<< 05a81d40e975ce23715891e36b8f9b95273d5f96

    }

    @Override public void onSystemReady() {
        try {
            IService<cav_srvs.GetDriversWithCapabilitiesRequest,
                cav_srvs.GetDriversWithCapabilitiesResponse> driverCapabilityService
                = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                GetDriversWithCapabilities._TYPE);

            GetDriversWithCapabilitiesRequest req =
                node.getServiceRequestMessageFactory()
                    .newFromType(GetDriversWithCapabilitiesRequest._TYPE);

            List<String> reqdCapabilities = new ArrayList<>();
            reqdCapabilities.add("lateral");
            reqdCapabilities.add("longitudinal");
            req.setCapabilities(reqdCapabilities);
            final GetDriversWithCapabilitiesResponse[] drivers =
                new GetDriversWithCapabilitiesResponse[1];
            driverCapabilityService.call(req,
                new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
                    @Override public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                        drivers[0] = msg;
                    }

                    @Override public void onFailure(Exception e) {
                        // Ignore
                    }
                });
        } catch (TopicNotFoundException e) {
            e.printStackTrace();
        }
    }

    @Override public void onGuidanceEnable() {
        // NO-OP
=======

        for (; ; ) {
            try {
                // Generate a simple sin(t) speed command
                commands.setCommand(Math.sin((System.currentTimeMillis() + phase) * frequency) * amplitude, maxAccel);
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
            }
        }
>>>>>>> Initial implementation of GuidanceCommands functionality
    }
}
