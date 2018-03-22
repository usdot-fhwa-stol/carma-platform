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
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.ConflictManager;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IMobilityTimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.SystemUTCTimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.MobilityRouter;
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
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.TrajectoryConverter;
import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

import org.apache.commons.logging.Log;
import org.ros.exception.ServiceException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
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
  protected final int NUMTHREADS = 8;
  protected static ComponentVersion version = CarmaVersion.getVersion();

  protected IPubSubService pubSubService;
  
  protected ConflictManager conflictManager;

  protected TrajectoryConverter trajectoryConverter;

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
    MobilityRouter router = new MobilityRouter(stateMachine, pubSubService, node, conflictManager, trajectoryConverter, trajectoryExecutor);
    PluginManager pluginManager = new PluginManager(
      stateMachine, pubSubService, guidanceCommands, maneuverInputs,
      routeService, node, router, conflictManager, trajectoryConverter);
    Arbitrator arbitrator = new Arbitrator(stateMachine, pubSubService, node, pluginManager, trajectoryExecutor);
    
    tracking.setTrajectoryExecutor(trajectoryExecutor);
    tracking.setArbitrator(arbitrator);
    trajectoryExecutor.setArbitrator(arbitrator);
    pluginManager.setArbitratorService(arbitrator);
    router.setPluginManager(pluginManager);

    executor.execute(stateHandler);
    executor.execute(maneuverInputs);
    executor.execute(arbitrator);
    executor.execute(pluginManager);
    executor.execute(trajectoryExecutor);
    executor.execute(tracking);
    executor.execute(guidanceCommands);
    executor.execute(router);
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

  /**
   * Initialize the Guidance conflict detection system
   * Must be called after initLogger to ensure logging is provided
   * Must be called before initExecutor
   */
  private void initConflictManager(ConnectedNode node, ILogger log) {
    // Load params
    ParameterTree params = node.getParameterTree();
    double cellDowntrack = params.getDouble("conflict_map_cell_downtrack_size", 5.0);
    double cellCrosstrack = params.getDouble("conflict_map_cell_crosstrack_size", 5.0);
    double cellTime = params.getDouble("conflict_map_cell_time_size", 0.15);
    
    double[] cellSize = {cellDowntrack, cellCrosstrack, cellTime};

    double downtrackMargin = params.getDouble("conflict_map_collision_downtrack_margin", 2.5);
    double crosstrackMargin = params.getDouble("conflict_map_collision_crosstrack_margin", 1.0);
    double timeMargin = params.getDouble("conflict_map_collision_time_margin", 0.05);
    // Echo params
    log.info("Param conflict_map_cell_downtrack_size: " + cellDowntrack);
    log.info("Param conflict_map_cell_crosstrack_size: " + cellCrosstrack);
    log.info("Param conflict_map_cell_time_size: " + cellTime);
    log.info("Param conflict_map_collision_downtrack_margin: " + downtrackMargin);
    log.info("Param conflict_map_collision_crosstrack_margin: " + crosstrackMargin);
    log.info("Param conflict_map_collision_time_margin: " + timeMargin);
    
    // Set time strategy
    IMobilityTimeProvider timeProvider = new SystemUTCTimeProvider();
    // Build conflict manager
    conflictManager = new ConflictManager(cellSize, downtrackMargin, crosstrackMargin, timeMargin, timeProvider);
  }

  /**
   * Initialize the Trajectory Conversion system for use in Mobility Messages
   * Must be called after initLogger to ensure logging is provided
   * Must be called before initExecutor
   */
  private void initTrajectoryConverter(ConnectedNode node, ILogger log) {
    // Load params
    ParameterTree params = node.getParameterTree();
    int maxPoints = params.getInteger("mobility_path_max_points", 60);
    double timeStep = params.getDouble("mobility_path_time_step", 0.1);
    // Echo params
    log.info("Param mobility_path_max_points: " + maxPoints);
    log.info("Param mobility_path_time_step: " + timeStep);
    
    // Build trajectory converter
    trajectoryConverter = new TrajectoryConverter(maxPoints, timeStep);
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

    initTrajectoryConverter(connectedNode, log);
    log.info("Guidance main TrajectoryConverter initialized");

    initConflictManager(connectedNode, log);
    log.info("Guidance main ConflictManager initialized");

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
