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

import gov.dot.fhwa.saxton.carma.guidance.arbitrator.Arbitrator;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import cav_srvs.GetSystemVersion;
import cav_srvs.GetSystemVersionRequest;
import cav_srvs.GetSystemVersionResponse;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.*;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.util.GuidanceRouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.guidance.util.SaxtonLoggerProxyFactory;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

import org.apache.commons.logging.Log;
import org.ros.exception.ServiceException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

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
  protected final int NUMTHREADS = 7;
  protected static ComponentVersion version = CarmaVersion.getVersion();

  protected IPubSubService pubSubService;

  protected GuidanceExceptionHandler exceptionHandler;

  protected final AtomicBoolean engaged = new AtomicBoolean(false);
  protected final AtomicBoolean systemReady = new AtomicBoolean(false);
  protected boolean initialized = false;
  
  ServiceServer<GetSystemVersionRequest, GetSystemVersionResponse> systemVersionServer;
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("guidance_main");
  }

  /**
   * Initialize the runnable thread members of the Guidance package.
   */
  private void initExecutor(GuidanceStateMachine stateMachine, ConnectedNode node) {
    executor = Executors.newFixedThreadPool(NUMTHREADS);

    // Init the Guidance component

    GuidanceRouteService routeService = new GuidanceRouteService(pubSubService);
    routeService.init();
    
    GuidanceStateHandler stateHandler = new GuidanceStateHandler(stateMachine, pubSubService, node);
    GuidanceCommands guidanceCommands = new GuidanceCommands(stateMachine, pubSubService, node);
    ManeuverInputs maneuverInputs = new ManeuverInputs(stateMachine, pubSubService, node);
    Tracking tracking = new Tracking(stateMachine, pubSubService, node);
    TrajectoryExecutor trajectoryExecutor = new TrajectoryExecutor(stateMachine, pubSubService, node, guidanceCommands, tracking);
    PluginManager pluginManager = new PluginManager(stateMachine, pubSubService, guidanceCommands, maneuverInputs, routeService, node);
    Arbitrator arbitrator = new Arbitrator(stateMachine, pubSubService, node, pluginManager, trajectoryExecutor);
    
    tracking.setTrajectoryExecutor(trajectoryExecutor);
    tracking.setArbitrator(arbitrator);
    trajectoryExecutor.setArbitrator(arbitrator);
    pluginManager.setArbitratorService(arbitrator);

    executor.execute(stateHandler);
    executor.execute(maneuverInputs);
    executor.execute(arbitrator);
    executor.execute(pluginManager);
    executor.execute(trajectoryExecutor);
    executor.execute(tracking);
    executor.execute(guidanceCommands);
  }

  /**
   * Initialize the PubSubManager and setup it's message queue.
   */
  private void initPubSubManager(ConnectedNode node, GuidanceExceptionHandler guidanceExceptionHandler) {
    ISubscriptionChannelFactory subscriptionChannelFactory = new RosSubscriptionChannelFactory(node, guidanceExceptionHandler);
    IPublicationChannelFactory publicationChannelFactory = new RosPublicationChannelFactory(node);
    IServiceChannelFactory serviceChannelFactory = new RosServiceChannelFactory(node, this);

    pubSubService = new PubSubManager(subscriptionChannelFactory, publicationChannelFactory, serviceChannelFactory);
  }

  /**
   * Initialize the Guidance logging system
   */
  private void initLogger(Log baseLog) {
    SaxtonLoggerProxyFactory slpf = new SaxtonLoggerProxyFactory(baseLog);
    LoggerManager.setLoggerFactory(slpf);
  }

  @Override
  public void onSaxtonStart(final ConnectedNode connectedNode) {
    initLogger(connectedNode.getLog());
    final ILogger log = LoggerManager.getLogger();
    
    log.info("//////////");
    log.info("//////////   GuidanceMain starting up:    " + version.toString() + "    //////////");
    log.info("//////////");

    final GuidanceStateMachine stateMachine = new GuidanceStateMachine();
    final GuidanceExceptionHandler guidanceExceptionHandler = new GuidanceExceptionHandler(stateMachine);
    log.info("Guidance exception handler initialized");

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
    
    stateMachine.initSubPub(pubSubService);
    
    initExecutor(stateMachine, connectedNode);
    log.info("Guidance main executor initialized");

    systemVersionServer = connectedNode.newServiceServer("get_system_version", GetSystemVersion._TYPE,
            new ServiceResponseBuilder<GetSystemVersionRequest, GetSystemVersionResponse>() {
                @Override
                public void build(GetSystemVersionRequest request, GetSystemVersionResponse response) throws ServiceException {
                    response.setSystemName(version.componentName());
                    response.setRevision(version.revisionString());
                }
            });
  }//onStart

  /**
   * Handle an exception that hasn't been caught anywhere else, which will cause guidance to shutdown.
   */
  @Override
  protected void handleException(Throwable e) {
    exceptionHandler.handleException(e);

    //Leverage SaxtonNode to publish the system alert.
    publishSystemAlert(AlertSeverity.FATAL, "Guidance PANIC triggered in thread " + Thread.currentThread().getName() + " by an uncaught exception!", e);
  }
}//AbstractNodeMain
