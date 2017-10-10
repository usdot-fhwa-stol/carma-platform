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
import cav_srvs.SetGuidanceEngaged;
import cav_srvs.SetGuidanceEngagedRequest;
import cav_srvs.SetGuidanceEngagedResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
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
  protected final int numThreads = 6;

  protected GuidanceExceptionHandler guidanceExceptionHandler;
  protected IPubSubService pubSubService;
  protected ServiceServer<SetGuidanceEngagedRequest, SetGuidanceEngagedResponse> guidanceEngageService;

  protected GuidanceExceptionHandler exceptionHandler;

  protected final AtomicBoolean engaged = new AtomicBoolean(false);
  protected final AtomicBoolean systemReady = new AtomicBoolean(false);
  protected boolean initialized = false;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("guidance_main");
  }

  /**
   * Initialize the runnable thread members of the Guidance package.
   */
  private void initExecutor(AtomicReference<GuidanceState> state, ConnectedNode node) {
    executor = Executors.newFixedThreadPool(numThreads);

    Arbitrator arbitrator = new Arbitrator(state, pubSubService, node);
    PluginManager pluginManager = new PluginManager(state, pubSubService, node);
    GuidanceCommands guidanceCommands = new GuidanceCommands(state, pubSubService, node);
    TrajectoryExecutor trajectoryExecutor = new TrajectoryExecutor(state, pubSubService, node, guidanceCommands);
    Tracking tracking = new Tracking(state, pubSubService, node);
    Maneuvers maneuvers = new Maneuvers(state, pubSubService, node);

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
  private void initPubSubManager(ConnectedNode node, GuidanceExceptionHandler guidance) {
    ISubscriptionChannelFactory subscriptionChannelFactory = new RosSubscriptionChannelFactory(node,
        guidanceExceptionHandler);
    IPublicationChannelFactory publicationChannelFactory = new RosPublicationChannelFactory(node);
    IServiceChannelFactory serviceChannelFactory = new RosServiceChannelFactory(node);

    pubSubService = new PubSubManager(subscriptionChannelFactory, publicationChannelFactory, serviceChannelFactory);
  }

  @Override
  public void onSaxtonStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    final AtomicReference<GuidanceState> state = new AtomicReference<>(GuidanceState.STARTUP);
    final GuidanceExceptionHandler guidanceExceptionHandler = new GuidanceExceptionHandler(state, log);
    this.exceptionHandler = guidanceExceptionHandler;
    log.info("Guidance exception handler partially initialized");

    // Allow GuidanceExceptionHandler to take over in the event a thread dies due to an uncaught exception
    // Will apply to any thread that lacks an otherwise specified ExceptionHandler
    // Not sure how this interacts with multiple processes sharing the same JVM
    Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
      @Override
      public void uncaughtException(Thread t, Throwable e) {
        log.fatal("Guidance thread " + t.getName() + " raised uncaught exception! Handling!!!");
        guidanceExceptionHandler.handleException(e);
      }
    });

    initPubSubManager(connectedNode, guidanceExceptionHandler);
    log.info("Guidance main PubSubManager initialized");

    guidanceExceptionHandler.init(pubSubService);
    log.info("Guidance main exception handler fully initialized");

    initExecutor(state, connectedNode);
    log.info("Guidance main executor initialized");

    // Currently setup to listen to it's own message. Change to listen to someone other topic.
    ISubscriber<SystemAlert> subscriber = pubSubService.getSubscriberForTopic("system_alert",
        cav_msgs.SystemAlert._TYPE);

    subscriber.registerOnMessageCallback(new OnMessageCallback<SystemAlert>() {
      @Override
      public void onMessage(cav_msgs.SystemAlert message) {
        if (message.getType() == SystemAlert.DRIVERS_READY) {
          state.set(GuidanceState.DRIVERS_READY);
          log.info("Guidance main received DRIVERS_READY!");
        } else if (message.getType() == SystemAlert.FATAL) {
          log.info("!!!!!! GuidanceMain received SystemAlert.FATAL! Shutting down! !!!!!");
          state.set(GuidanceState.SHUTDOWN);
        } else if (message.getType() == SystemAlert.SHUTDOWN) {
          log.info("GuidanceMain received shutdown command. Shutting down!");
          state.set(GuidanceState.SHUTDOWN);
        }
      }//onNewMessage
    } //MessageListener
    );//addMessageListener

    final IPublisher<SystemAlert> systemAlertPublisher = pubSubService.getPublisherForTopic("system_alert",
        cav_msgs.SystemAlert._TYPE);

    guidanceEngageService = connectedNode.newServiceServer("set_guidance_engaged", SetGuidanceEngaged._TYPE,
        new ServiceResponseBuilder<SetGuidanceEngagedRequest, SetGuidanceEngagedResponse>() {
          @Override
          public void build(SetGuidanceEngagedRequest setGuidanceEngagedRequest,
              SetGuidanceEngagedResponse setGuidanceEngagedResponse) throws ServiceException {
            state.set(GuidanceState.ENGAGED);
            setGuidanceEngagedResponse.setGuidanceStatus(state.get() == GuidanceState.ENGAGED);
          }
        });

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        Thread.sleep(5000);
      }//loop
    }//CancellableLoop
    );//executeCancellableLoop
  }//onStart

  @Override
  protected void handleException(Throwable e) {
    exceptionHandler.handleException(e);
  }
}//AbstractNodeMain
