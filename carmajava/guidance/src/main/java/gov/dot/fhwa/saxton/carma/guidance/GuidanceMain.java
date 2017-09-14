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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import cav_srvs.SetGuidanceEnabled;
import cav_srvs.SetGuidanceEnabledRequest;
import cav_srvs.SetGuidanceEnabledResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The top-level Guidance package is responsible for providing basic facilities needed by all elements of
 * the Guidance package. It forms the Guidance ROS node.
 * <p>
 * Command line test: rosrun carma guidance gov.dot.fhwa.saxton.carma.guidance.GuidanceMain
 * rosservice call /plugins/getRegisteredPlugins
 * rosservice call /plugins/getActivePlugins
 * rosservice call /plugins/getAvailablePlugins
 * rosservice call /plugins/activatePlugin '{header: auto, pluginName: DUMMY PLUGIN A, pluginVersion: v2.0.0, activated: True}'
 */
public class GuidanceMain extends SaxtonBaseNode {

    // Member Variables
    protected ExecutorService executor;
    protected int numThreads = 6;

    protected IPubSubService pubSubService;
    protected ServiceServer<SetGuidanceEnabledRequest, SetGuidanceEnabledResponse>
        guidanceEnableService;

    protected final AtomicBoolean enabled = new AtomicBoolean(false);
    protected final AtomicBoolean systemReady = new AtomicBoolean(false);
    protected boolean initialized = false;

    @Override public GraphName getDefaultNodeName() {
        return GraphName.of("guidance_main");
    }

    /**
     * Initialize the runnable thread members of the Guidance package.
     */
    private void initExecutor(AtomicReference<GuidanceState> state, ConnectedNode node) {
        executor = Executors.newFixedThreadPool(numThreads);

        Arbitrator arbitrator = new Arbitrator(pubSubService, node);
        PluginManager pluginManager = new PluginManager(pubSubService, node);
        TrajectoryExecutor trajectoryExecutor = new TrajectoryExecutor(pubSubService, node);
        Tracking tracking = new Tracking(pubSubService, node);
        GuidanceCommands guidanceCommands = new GuidanceCommands(pubSubService, node);
        Maneuvers maneuvers = new Maneuvers(pubSubService, node);

        executor.execute(maneuvers);
        executor.execute(arbitrator);
        executor.execute(pluginManager);
        executor.execute(trajectoryExecutor);
        executor.execute(tracking);
        executor.execute(guidanceCommands);
    }

    /**
     * Initialize the PubSubManager and setup it's message queue.
     */
    private void initPubSubManager(ConnectedNode node) {
        ISubscriptionChannelFactory subscriptionChannelFactory =
            new RosSubscriptionChannelFactory(node);
        IPublicationChannelFactory publicationChannelFactory =
            new RosPublicationChannelFactory(node);
        IServiceChannelFactory serviceChannelFactory = new RosServiceChannelFactory(node);

        pubSubService = new PubSubManager(subscriptionChannelFactory, publicationChannelFactory,
            serviceChannelFactory);
    }

    @Override public void onStart(final ConnectedNode connectedNode) {
        final AtomicReference<GuidanceState> state = new AtomicReference<>(GuidanceState.STARTUP);

        // Configure the comms classes
        final Log log = connectedNode.getLog();

        initPubSubManager(connectedNode);
        log.info("Guidance main PubSubManager initialized");
        initExecutor(state, connectedNode);
        log.info("Guidance main executor initialized");

        // Currently setup to listen to it's own message. Change to listen to someone other topic.
        ISubscriber<SystemAlert> subscriber =
            pubSubService.getSubscriberForTopic("system_alert", cav_msgs.SystemAlert._TYPE);

        subscriber.registerOnMessageCallback(new OnMessageCallback<SystemAlert>() {
                                                 @Override public void onMessage(cav_msgs.SystemAlert message) {
                                                     if (message.getType() == SystemAlert.SYSTEM_READY) {
                                                         state.set(GuidanceState.DRIVERS_READY);
                                                         log.info("Guidance main received DRIVERS_READY!");
                                                     }
                                                 }//onNewMessage
                                             }//MessageListener
        );//addMessageListener

        final IPublisher<SystemAlert> systemAlertPublisher =
            pubSubService.getPublisherForTopic("system_alert", cav_msgs.SystemAlert._TYPE);

        guidanceEnableService = connectedNode.newServiceServer("set_guidance_enabled",
            SetGuidanceEnabled._TYPE,
            new ServiceResponseBuilder<SetGuidanceEnabledRequest, SetGuidanceEnabledResponse>() {
                @Override public void build(SetGuidanceEnabledRequest setGuidanceEnabledRequest,
                    SetGuidanceEnabledResponse setGuidanceEnabledResponse) throws ServiceException {
                    state.set(GuidanceState.ENABLED);
                    setGuidanceEnabledResponse.setGuidanceStatus(enabled.get());
                }
            });


        // Primary GuidanceMain loop logic
        //Getting the ros param called run_id.
        ParameterTree param = connectedNode.getParameterTree();
        final String rosRunID = param.getString("/run_id");

        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
                                                 private int sequenceNumber;

                                                 @Override protected void setup() {
                                                     sequenceNumber = 0;
                                                 }//setup

                                                 @Override protected void loop() throws InterruptedException {

                                                     cav_msgs.SystemAlert systemAlertMsg = systemAlertPublisher.newMessage();
                                                     systemAlertMsg.setDescription(
                                                         "Hello World! " + "I am guidance_main. " + sequenceNumber + " run_id = "
                                                             + rosRunID + ".");
                                                     systemAlertMsg.setType(SystemAlert.CAUTION);
                                                     systemAlertPublisher.publish(systemAlertMsg);
                                                     sequenceNumber++;

                                                     Thread.sleep(30000);
                                                 }//loop

                                             }//CancellableLoop
        );//executeCancellableLoop
    }//onStart
}//AbstractNodeMain
