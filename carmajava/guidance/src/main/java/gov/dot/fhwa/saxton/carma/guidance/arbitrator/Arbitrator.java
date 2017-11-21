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

package gov.dot.fhwa.saxton.carma.guidance.arbitrator;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import com.google.common.util.concurrent.AtomicDouble;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.plugins.CruisingPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.GlobalSpeedLimitConstraint;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.LocalSpeedLimitConstraint;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.OnTrajectoryProgressCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryValidationConstraint;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryValidator;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Arbitrator component
 * <p>
 * Runs inside the GuidanceMain class's executor and prompts the Guidance.Plugins
 * to plan trajectories for the vehicle to execute.
 */
public class Arbitrator extends GuidanceComponent implements ArbitratorStateChangeListener {
  protected ArbitratorStateMachine stateMachine = new ArbitratorStateMachine();
  protected ISubscriber<RouteState> routeStateSubscriber;
  protected ISubscriber<TwistStamped> twistSubscriber;
  protected AtomicReference<GuidanceState> state;
  protected AtomicBoolean needsReplan = new AtomicBoolean(false);
  protected PluginManager pluginManager;
  protected IPlugin lateralPlugin;
  protected IPlugin longitudinalPlugin;
  protected CruisingPlugin cruisingPlugin;
  protected AtomicBoolean receivedDtdUpdate = new AtomicBoolean(false);
  protected AtomicDouble downtrackDistance = new AtomicDouble(0.0);
  protected AtomicDouble currentSpeed = new AtomicDouble(0.0);
  protected double replanTriggerPercent = 0.75;
  protected double complexTrajectoryReplanTriggerPercent = 0.95;
  protected double minimumTrajectoryLength = 50.0;
  protected double planningWindow = 0.0;
  protected double planningWindowGrowthFactor = 0.0;
  protected double planningWindowShrinkFactor = 0.0;
  protected double planningWindowSnapThreshold = 20.0;
  protected double postComplexSteadyingDuration = 2.0;
  protected int numAcceptableFailures = 0;
  protected Trajectory currentTrajectory = null;
  protected TrajectoryValidator trajectoryValidator;
  protected TrajectoryExecutor trajectoryExecutor;
  protected String lateralPluginName = "NoOp Plugin";
  protected String longitudinalPluginName = "Cruising Plugin";
  protected ISubscriber<Route> routeSub;
  protected AtomicDouble routeLength = new AtomicDouble(-1.0);
  protected AtomicReference<Route> currentRoute = new AtomicReference<>();
  protected AtomicBoolean routeRecvd = new AtomicBoolean(false);
  protected BlockingQueue<Runnable> jobQueue = new LinkedBlockingQueue<>();
  protected static final long SLEEP_DURATION_MILLIS = 100;
  protected static final double TRAJ_SIZE_WARNING = 50.0;

  Arbitrator(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node,
      PluginManager pluginManager, TrajectoryExecutor trajectoryExecutor) {
    super(state, iPubSubService, node);
    this.state = state;
    this.pluginManager = pluginManager;
    this.trajectoryValidator = new TrajectoryValidator();
    this.trajectoryExecutor = trajectoryExecutor;
  }

  /**
   * Instantiate a list of constraint classes into live objects
   * @param classes The list of classes which implement {@link TrajectoryValidationConstraint}
   * @return A list containing instantiated TrajectoryValidationConstraint instances where the instantiation was successful
   */
  protected List<TrajectoryValidationConstraint> instantiateConstraints(
      List<Class<? extends TrajectoryValidationConstraint>> classes) {
    List<TrajectoryValidationConstraint> constraintInstances = new ArrayList<>();
    for (Class<? extends TrajectoryValidationConstraint> constraintClass : classes) {
      try {
        Constructor<? extends TrajectoryValidationConstraint> constraintCtor = constraintClass.getConstructor();

        // TODO: This is brittle, depends on convention of having a constructor taking no arguments
        TrajectoryValidationConstraint constraintInstance = constraintCtor.newInstance();
        log.info("Guidance.Arbitrator instantiated new TrajectoryValidationConstraint instance: "
            + constraintClass.getCanonicalName());

        constraintInstances.add(constraintInstance);
      } catch (Exception e) {
        log.error("Unable to instantiate: " + constraintClass.getCanonicalName(), e);
      }
    }

    return constraintInstances;
  }

  @Override
  public void onGuidanceStartup() {
    log.info("STARTUP", "Arbitrator running!");
    routeStateSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
      @Override
      public void onMessage(RouteState msg) {
        log.info("Received RouteState:" + msg);
        downtrackDistance.set(msg.getDownTrack());
      }
    });

    ParameterTree ptree = node.getParameterTree();
    replanTriggerPercent = ptree.getDouble("~arbitrator_replan_threshold", 0.75);
    planningWindow = ptree.getDouble("~initial_planning_window", 10.0);
    planningWindowGrowthFactor = ptree.getDouble("~planning_window_growth_factor", 1.0);
    planningWindowShrinkFactor = ptree.getDouble("~planning_window_shrink_factor", 1.0);
    numAcceptableFailures = ptree.getInteger("~trajectory_planning_max_attempts", 3);
    longitudinalPluginName = ptree.getString("~arbitrator_longitudinal_plugin");
    lateralPluginName = ptree.getString("~arbitrator_lateral_plugin");
    planningWindowSnapThreshold = ptree.getDouble("~planning_window_snap_threshold", 20.0);
    postComplexSteadyingDuration = ptree.getDouble("~post_complex_trajectory_steadying_period", 2.0);
    double configuredSpeedLimit = ptree.getDouble("~trajectory_speed_limit", GuidanceCommands.MAX_SPEED_CMD_M_S);

    routeSub = pubSubService.getSubscriberForTopic("route", Route._TYPE);
    routeSub.registerOnMessageCallback(new OnMessageCallback<Route>() {
      @Override
      public void onMessage(Route msg) {
        if (!routeRecvd.get()) {
          log.info("Arbitrator now using LocalSpeedLimit constraint for route " + msg.getRouteName());
          trajectoryValidator.addValidationConstraint(new LocalSpeedLimitConstraint(msg)); routeRecvd.set(true);
          currentRoute.set(msg);

          double length = 0.0;
          for (RouteSegment segment : msg.getSegments()) {
            length += segment.getLength();
          }
          routeLength.set(length);
          log.info("Computed total route length to be " + routeLength.get());
        }
      }
    });

    // Instantiate the configured constraints and register them with the TrajectoryValidator
    List<String> constraintNames = (List<String>) ptree.getList("~trajectory_constraints");
    List<Class<? extends TrajectoryValidationConstraint>> constraintClasses = new ArrayList<>();
    for (String className : constraintNames) {
      try {
        constraintClasses.add((Class<? extends TrajectoryValidationConstraint>) Class.forName(className));
      } catch (Exception e) {
        log.warn("STARTUP", "Unable to get Class object for name: " + className);
      }
    }

    List<TrajectoryValidationConstraint> constraints = instantiateConstraints(constraintClasses);
    for (TrajectoryValidationConstraint tvc : constraints) {
      trajectoryValidator.addValidationConstraint(tvc);
      log.info("STARTUP", "Aribtrator using TrajectoryValidationConstraint: " + tvc.getClass().getSimpleName());
    }

    constraints.add(new GlobalSpeedLimitConstraint(configuredSpeedLimit));
    log.info("Arbitrator using GlobalSpeedLimitConstraint with limit: " + configuredSpeedLimit + " m/s");

    twistSubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
    twistSubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
      @Override
      public void onMessage(TwistStamped msg) {
        currentSpeed.set(msg.getTwist().getLinear().getX());
        receivedDtdUpdate.set(true);
      }
    });
  }

  @Override
  public String getComponentName() {
    return "Guidance.Arbitrator";
  }

  @Override
  public void onSystemReady() {
    // NO-OP
  }

  @Override
  public void onGuidanceEnable() {
    // For now, find the configured lateral and longitudinal plugins
    for (IPlugin plugin : pluginManager.getRegisteredPlugins()) {
      if (plugin.getVersionInfo().componentName().equals(lateralPluginName)) {
        lateralPlugin = plugin;
      }

      if (plugin.getVersionInfo().componentName().equals(longitudinalPluginName)) {
        longitudinalPlugin = plugin;
      }

      if (plugin instanceof CruisingPlugin) {
        cruisingPlugin = (CruisingPlugin) plugin;
      }
    }

    if (lateralPlugin == null || longitudinalPlugin == null) {
      panic("Arbitrator unable to locate the configured and required plugins!");
    }

    log.info("PLUGIN", "Arbitrator using plugins: [" + lateralPluginName + ", " + longitudinalPluginName + "]");
    stateMachine.processEvent(ArbitratorEvent.INITIALIZE);
  }

  /**
   * Compute the endpoint of the trajectory starting at trajectoryStart.
   * <p>
   * Snaps the end of the trajectory to the end of route segments within planningWindowSnapThreshold
   * of the end of the planning window.
   * Also ensures that the trajectory does not exceed the route length
   * 
   * @param trajectoryStart the start point of the desired trajectory
   * @return The end of the trajectory adjusted as described above
   */
  private double getNextTrajectoryEndpoint(double trajectoryStart) {
    double trajectoryEnd = trajectoryStart + planningWindow;

    // Examine our current route to determine if there is an acceptable segment to snap to
    if (routeLength.get() > 0.0) {
      double dtdAccum = 0.0;
      for (RouteSegment segment : currentRoute.get().getSegments()) {
        dtdAccum += segment.getLength();
      
        if (dtdAccum > trajectoryEnd) {
          if (Math.abs(dtdAccum - trajectoryEnd) < planningWindowSnapThreshold) {
            trajectoryEnd = dtdAccum;
          } else {
            break;
          }
        }
      }

      trajectoryEnd = Math.min(routeLength.get(), trajectoryEnd);
    } 

    return trajectoryEnd;
  }

  @Override
  public void loop() throws InterruptedException {
    // Simple job-queue loop with jobs being added based on state machine transitions
    log.info("Arbitrator polling job queue...");
    Runnable job = jobQueue.take();

    log.info("Arbitrator found job of type: " + job.getClass().getSimpleName() + "! Executing...");
    job.run();
  }

  protected void increasePlanningWindow() {
    planningWindow *= planningWindowGrowthFactor;
  }

  protected void decreasePlanningWindow() {
    planningWindow *= planningWindowShrinkFactor;
    planningWindow = Math.max(planningWindow, minimumTrajectoryLength);
  }

  protected Trajectory planTrajectory(double trajectoryStart, double trajectoryEnd) {
    log.info("Arbitrator planning new trajectory spanning [" + trajectoryStart + ", " + trajectoryEnd
        + ")");


    if (trajectoryEnd - trajectoryStart < TRAJ_SIZE_WARNING) {
      log.warn("Trajectory planned smaller than " + TRAJ_SIZE_WARNING + ". Maneuvers may not have space to complete.");
    }

    Trajectory out = null;
    for (int failures = 0; failures < numAcceptableFailures; failures++) {
      Trajectory traj = new Trajectory(trajectoryStart, trajectoryEnd);
      double expectedEntrySpeed = 0.0;
      if (currentTrajectory != null) {
        List<LongitudinalManeuver> lonManeuvers = currentTrajectory.getLongitudinalManeuvers();
        LongitudinalManeuver lastManeuver = lonManeuvers.get(lonManeuvers.size() - 1);
        expectedEntrySpeed = lastManeuver.getTargetSpeed();
      } else {
        expectedEntrySpeed = currentSpeed.get();
      }

      lateralPlugin.planTrajectory(traj, expectedEntrySpeed);
      longitudinalPlugin.planTrajectory(traj, expectedEntrySpeed);
      if (trajectoryValidator.validate(traj)) {
        out = traj;
        break;
      }
      log.warn("Candidate trajectory #" + (failures + 1) + " failed validation.");
    }

    if (out == null) {
      panic("Arbitrator unable to plan valid trajectory after " + numAcceptableFailures + " attempts!");
    }

    return out;
  }

  /**
   * Job queue task for performing the planning of the first Trajectory
   */
  private class InitialPlanningTask implements Runnable {

    @Override
    public void run() {
      log.info("Arbitrator performing initial trajectory planning");
      // Wait until we've received a downtrack distance update to try to plan our first trajectory
      if (!receivedDtdUpdate.get()) {
        log.info("Arbitrator waiting for DTD update from Route...");
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
        }
      }

      double trajectoryStart = downtrackDistance.get();
      currentTrajectory = planTrajectory(downtrackDistance.get(), getNextTrajectoryEndpoint(trajectoryStart));
      trajectoryExecutor.registerOnTrajectoryProgressCallback(replanTriggerPercent, 
          (pct) -> {
            if (trajectoryExecutor.getCurrentTrajectory() != null
                && trajectoryExecutor.getCurrentTrajectory().getComplexManeuver() != null) {
              OnTrajectoryProgressCallback complexReplanCallback = new OnTrajectoryProgressCallback() {
                @Override
                public void onProgress(double pct) {
                  // Ensure we're only called once
                  trajectoryExecutor.unregisterOnTrajectoryProgressCallback(this);

                  // Trigger the replan
                  stateMachine.processEvent(ArbitratorEvent.COMPLEX_TRAJECTORY_COMPLETION_ALERT);
                }
              };

              // Schedule this to execute closer to the end of the complex trajectory
              trajectoryExecutor.registerOnTrajectoryProgressCallback(complexTrajectoryReplanTriggerPercent, complexReplanCallback);
            } else {
              stateMachine.processEvent(ArbitratorEvent.TRAJECTORY_COMPLETION_ALERT);
            }
      });

      trajectoryExecutor.runTrajectory(currentTrajectory);
    }

  }

  /**
   * Job queue task for performing replanning after successful completion of the
   * previous trajectory
   */
  private class NormalReplanningTask implements Runnable {

    @Override
    public void run() {
      log.info("Arbitrator running normal replanning process");
      if (downtrackDistance.get() < routeLength.get()) {
        increasePlanningWindow();

        double trajectoryStart = Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation());
        double trajectoryEnd = getNextTrajectoryEndpoint(trajectoryStart);

        currentTrajectory = planTrajectory(trajectoryStart, trajectoryEnd);
        trajectoryExecutor.runTrajectory(currentTrajectory);
      } else {
        log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
      }
    }

  }

  /**
   * Job queue task for performing replanning after the execution of a complex maneuver
   * <p>
   * Plans temporary steadying trajectory for a configured duration then replans as usual
   */
  private class ComplexTrajectoryReplanTask implements Runnable {

    @Override
    public void run() {
      log.info("Arbitrator replanning trajectory after complex maneuver execution");
      if (downtrackDistance.get() < routeLength.get()) {
        increasePlanningWindow();
        
        double steadyingTrajectoryStart = Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation());
        double steadyingTrajectoryEnd = steadyingTrajectoryStart + (postComplexSteadyingDuration * currentSpeed.get());
        Trajectory steadyingTrajectory = new Trajectory(steadyingTrajectoryStart, steadyingTrajectoryEnd);
        
        cruisingPlugin.planTrajectory(steadyingTrajectory, currentSpeed.get());
        trajectoryExecutor.runTrajectory(steadyingTrajectory);
        currentTrajectory = steadyingTrajectory;

        // Begin normal trajectory replanning immediately
        NormalReplanningTask normalReplanningTask = new NormalReplanningTask();
        normalReplanningTask.run();
      } else {
        log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
      }

    }

  }

  /**
   * Job queue task for replanning after the Tracking component has notified us of
   * trajectory failure.
   */
  private class FailedTrajectoryReplanTask implements Runnable {

    @Override
    public void run() {
      log.info("Arbitrator replanning trajectory because of failed execution!");
      if (downtrackDistance.get() < routeLength.get()) {
        decreasePlanningWindow();

        double trajectoryStart = Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation());
        double trajectoryEnd = getNextTrajectoryEndpoint(trajectoryStart);

        currentTrajectory = planTrajectory(trajectoryStart, trajectoryEnd);
        trajectoryExecutor.abortTrajectory();
        trajectoryExecutor.runTrajectory(currentTrajectory);
      } else {
        log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
      }
    }

  }

  /**
   * Inform the Arbitrator of trajectory execution failure, necessitating immediate replan and
   * execution of a new trajectory.
   */
  public void notifyTrajectoryFailure() {
    stateMachine.processEvent(ArbitratorEvent.TRAJECTORY_FAILED_EXECUTION);
  }

  @Override
  public void onStateChange(ArbitratorState newState) {
    switch (newState) {
      case INITIAL_PLANNING:
        jobQueue.add(new InitialPlanningTask());
        break;
      case AWAITING_REPLAN:
        // Don't submit anything to the job queue
        break;
      case NORMAL_REPLANNING:
        jobQueue.add(new NormalReplanningTask());
        break;
      case REPLAN_AFTER_COMPLEX_TRAJECTORY:
        jobQueue.add(new ComplexTrajectoryReplanTask());
        break;
      case REPLAN_DUE_TO_FAILED_TRAJECTORY:
        jobQueue.add(new FailedTrajectoryReplanTask());
        break;
      default:
        throw new IllegalStateException("Unrecognized arbitrator state detected!");
    }
  }
}
