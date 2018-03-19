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

package gov.dot.fhwa.saxton.carma.guidance.arbitrator;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import com.google.common.util.concurrent.AtomicDouble;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceAction;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.IStateChangeListener;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse.PlanningRequest;
import gov.dot.fhwa.saxton.carma.guidance.cruising.CruisingPlugin;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
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

import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Arbitrator component
 * <p>
 * Runs inside the GuidanceMain class's executor and prompts the Guidance.Plugins
 * to plan trajectories for the vehicle to execute.
 */
public class Arbitrator extends GuidanceComponent
    implements ArbitratorService, IStateChangeListener, ArbitratorStateChangeListener {
  protected ArbitratorStateMachine arbitratorStateMachine = new ArbitratorStateMachine();
  protected ISubscriber<RouteState> routeStateSubscriber;
  protected ISubscriber<TwistStamped> twistSubscriber;
  protected AtomicReference<GuidanceState> state;
  protected PluginManager pluginManager;
  protected List<IStrategicPlugin> plugins = new ArrayList<>();
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
  protected Trajectory trajectory = null;
  protected TrajectoryValidator trajectoryValidator;
  protected TrajectoryExecutor trajectoryExecutor;
  protected List<String> pluginNames = new ArrayList<>();
  protected ISubscriber<Route> routeSub;
  protected AtomicDouble routeLength = new AtomicDouble(-1.0);
  protected AtomicReference<Route> currentRoute = new AtomicReference<>();
  protected AtomicBoolean routeRecvd = new AtomicBoolean(false);
  protected static final long SLEEP_DURATION_MILLIS = 100;
  protected static final double TRAJ_SIZE_WARNING = 50.0;
  protected static final double DISTANCE_EPSILON = 0.0001;

  public Arbitrator(GuidanceStateMachine stateMachine, IPubSubService iPubSubService, ConnectedNode node,
      PluginManager pluginManager, TrajectoryExecutor trajectoryExecutor) {
    super(stateMachine, iPubSubService, node);
    this.pluginManager = pluginManager;
    this.trajectoryValidator = new TrajectoryValidator();
    this.trajectoryExecutor = trajectoryExecutor;
    jobQueue.add(this::onStartup);
    arbitratorStateMachine.registerStateChangeListener(this);
    stateMachine.registerStateChangeListener(this);

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
  public void onStartup() {
    log.info("STARTUP", "Arbitrator running!");
    routeStateSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSubscriber.registerOnMessageCallback((msg) -> {
      log.info("Received RouteState:" + msg);
      downtrackDistance.set(msg.getDownTrack());
      receivedDtdUpdate.set(true);
    });

    ParameterTree ptree = node.getParameterTree();
    replanTriggerPercent = ptree.getDouble("~arbitrator_replan_threshold", 0.75);
    planningWindow = ptree.getDouble("~initial_planning_window", 10.0);
    planningWindowGrowthFactor = ptree.getDouble("~planning_window_growth_factor", 1.0);
    planningWindowShrinkFactor = ptree.getDouble("~planning_window_shrink_factor", 1.0);
    numAcceptableFailures = ptree.getInteger("~trajectory_planning_max_attempts", 3);
    pluginNames = (List<String>) ptree.getList("~arbitrator_plugins");
    planningWindowSnapThreshold = ptree.getDouble("~planning_window_snap_threshold", 20.0);
    postComplexSteadyingDuration = ptree.getDouble("~post_complex_trajectory_steadying_period", 2.0);
    double configuredSpeedLimit = ptree.getDouble("~trajectory_speed_limit", GuidanceCommands.MAX_SPEED_CMD_M_S);

    routeSub = pubSubService.getSubscriberForTopic("route", Route._TYPE);
    routeSub.registerOnMessageCallback(new OnMessageCallback<Route>() {
      @Override
      public void onMessage(Route msg) {
        if (!routeRecvd.get()) {
          log.info("Arbitrator now using LocalSpeedLimit constraint for route " + msg.getRouteName());
          trajectoryValidator.addValidationConstraint(new LocalSpeedLimitConstraint(msg));
          routeRecvd.set(true);
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

    // Instantiate the configured constraints and register them with the
    // TrajectoryValidator
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
    constraints.add(new GlobalSpeedLimitConstraint(configuredSpeedLimit));
    log.info("Arbitrator using GlobalSpeedLimitConstraint with limit: " + configuredSpeedLimit + " m/s");

    for (TrajectoryValidationConstraint tvc : constraints) {
      trajectoryValidator.addValidationConstraint(tvc);
      log.info("STARTUP", "Aribtrator using TrajectoryValidationConstraint: " + tvc.getClass().getSimpleName());
    }

    twistSubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
    twistSubscriber.registerOnMessageCallback((msg) -> {
      currentSpeed.set(msg.getTwist().getLinear().getX());
    });

    currentState.set(GuidanceState.STARTUP);
  }

  @Override
  public void onSystemReady() {
    currentState.set(GuidanceState.DRIVERS_READY);
  }

  @Override
  public void onActive() {
    // For now, find the configured lateral and longitudinal plugins
    for (IPlugin plugin : pluginManager.getRegisteredPlugins()) {
      if (plugin instanceof CruisingPlugin) {
        setCruisingPlugin((CruisingPlugin) plugin);
      }
    }

    for (String name : pluginNames) {
      for (IPlugin p : pluginManager.getRegisteredPlugins()) {
        if (p.getVersionInfo().componentName().equals(name) && p instanceof IStrategicPlugin) {
          addStrategicPlugin((IStrategicPlugin) p);
        }
      }
    }

    if (plugins.isEmpty()) {
      exceptionHandler.handleException("Arbitrator unable to locate any plugins!",
          new RosRuntimeException("No plugins"));
    }

    String pluginList = "";
    for (IPlugin p : plugins) {
      pluginList += ", " + p.getVersionInfo().componentName();
    }

    log.info("PLUGIN", "Arbitrator using plugins: [" + pluginList + "]");

    currentState.set(GuidanceState.ACTIVE);
  }

  @Override
  public void onDeactivate() {
    currentState.set(GuidanceState.INACTIVE);
  }

  @Override
  public void onEngaged() {
    arbitratorStateMachine.processEvent(ArbitratorEvent.INITIALIZE);
    currentState.set(GuidanceState.ENGAGED);
  }

  @Override
  public void onCleanRestart() {
    arbitratorStateMachine.processEvent(ArbitratorEvent.CLEAN_RESTART);
    // Reset member variables
    cruisingPlugin = null;
    plugins.clear();
    trajectory = null;
    planningWindow = node.getParameterTree().getDouble("~initial_planning_window", 10.0);
    receivedDtdUpdate.set(false);

    ParameterTree ptree = node.getParameterTree();
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
    double configuredSpeedLimit = ptree.getDouble("~trajectory_speed_limit", GuidanceCommands.MAX_SPEED_CMD_M_S);
    constraints.add(new GlobalSpeedLimitConstraint(configuredSpeedLimit));
    log.info("Arbitrator using GlobalSpeedLimitConstraint with limit: " + configuredSpeedLimit + " m/s");

    trajectoryValidator = new TrajectoryValidator();
    for (TrajectoryValidationConstraint tvc : constraints) {
      trajectoryValidator.addValidationConstraint(tvc);
      log.info("Aribtrator using TrajectoryValidationConstraint: " + tvc.getClass().getSimpleName());
    }
    routeRecvd.set(false);
    currentState.set(GuidanceState.DRIVERS_READY);
  }

  @Override
  public String getComponentName() {
    return "Guidance.Arbitrator";
  }

  protected void addStrategicPlugin(IStrategicPlugin plugin) {
    plugins.add(plugin);
  }

  protected void setCruisingPlugin(CruisingPlugin plugin) {
    cruisingPlugin = plugin;
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

  protected void increasePlanningWindow() {
    planningWindow *= planningWindowGrowthFactor;
  }

  protected void decreasePlanningWindow() {
    planningWindow *= planningWindowShrinkFactor;
    planningWindow = Math.max(planningWindow, minimumTrajectoryLength);
  }

  protected Trajectory planTrajectory(double trajectoryStart, double trajectoryEnd) {
    long planningStart = System.currentTimeMillis();
    log.info("Arbitrator planning new trajectory spanning [" + trajectoryStart + ", " + trajectoryEnd + ")");

    if (trajectoryEnd - trajectoryStart < TRAJ_SIZE_WARNING) {
      log.warn("Trajectory planned smaller than " + TRAJ_SIZE_WARNING + ". Maneuvers may not have space to complete.");
    }

    Trajectory out = null;
    planningLoop: for (int failures = 0; failures < numAcceptableFailures; failures++) {
      Trajectory traj = new Trajectory(trajectoryStart, trajectoryEnd);
      double expectedEntrySpeed = 0.0;
      if (trajectory != null) {
        if (trajectory.getComplexManeuver() != null) {
          expectedEntrySpeed = currentSpeed.get();
        } else {
          List<LongitudinalManeuver> lonManeuvers = trajectory.getLongitudinalManeuvers();
          LongitudinalManeuver lastManeuver = lonManeuvers.get(lonManeuvers.size() - 1);
          expectedEntrySpeed = lastManeuver.getTargetSpeed();
        }
      } else {
        expectedEntrySpeed = currentSpeed.get();
      }

      // Use temp list to allow for modification
      List<IStrategicPlugin> tmpPlugins = new ArrayList<>(plugins);
      for (IStrategicPlugin p : tmpPlugins) {
        if (p.getActivation() && p.getAvailability()) {
          log.info("Allowing plugin: " + p.getVersionInfo().componentName() + " to plan trajectory.");
          TrajectoryPlanningResponse resp = ((IStrategicPlugin) p).planTrajectory(traj, expectedEntrySpeed);

          // Process the plugin's requests
          if (!resp.getRequests().isEmpty()) {
            // Update downtrack end of trajectory if requested
            if (resp.getProposedTrajectoryEnd().isPresent()) {
              double proposedEndDistance = resp.getProposedTrajectoryEnd().get();
              trajectoryEnd = Math.max(proposedEndDistance, trajectoryEnd);
              log.info("Candidate trajectory #" + (failures + 1) + " Plugin: " + p.getVersionInfo().componentName()
                  + " requested extended trajectory to " + trajectoryEnd);
            }

            // Sleep for replan delay if requested
            final int numFails = failures; // Copy into final var so lambda can read it
            resp.getProposedReplanDelay().ifPresent((delay) -> {
              log.info("Candidate trajectory #" + (numFails + 1) + " Plugin: " + p.getVersionInfo().componentName()
                  + " requested to delay planning for " + delay + " ms");

              try {
                Thread.sleep(delay);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            });

            // Promote the plugin to the top of the line for the next iteration
            if (resp.higherPriorityRequested()) {
              log.info("Candidate trajectory #" + (failures + 1) + " Plugin: " + p.getVersionInfo().componentName()
                  + " requested higher priority");
              plugins.remove(p);
              plugins.add(0, p);
            }

            // Increment planning failures and skip to next iteration of outer loop
            failures++;
            continue planningLoop; // Jump back to loop labled "planningLoop"
          }
        }
      }

      if (trajectoryValidator.validate(traj)) {
        out = traj;
        break;
      }
      log.warn("Candidate trajectory #" + (failures + 1) + " failed validation.");
    }

    if (out == null) {
      exceptionHandler.handleException(
          "Arbitrator unable to plan valid trajectory after " + numAcceptableFailures + " attempts!",
          new RosRuntimeException("Unable to plan trajectory"));
    }

    long planningEnd = System.currentTimeMillis();
    log.info("New trajectory planned in " + (planningEnd - planningStart) + " ms. Planning finished at "
        + trajectoryExecutor.getTrajectoryCompletionPct() + "%");

    return out;
  }

  /**
   * Job queue task for performing the planning of the first Trajectory
   */
  private void initialPlan() {
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

    if (Math.abs(trajectoryStart - routeLength.get()) < DISTANCE_EPSILON) {
      // Trajectory would start after route ends just don't do anything
      // This case REALLLY should never happen, but just for completeness...
      arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
      return;
    }

    trajectory = planTrajectory(downtrackDistance.get(), getNextTrajectoryEndpoint(trajectoryStart));
    trajectoryExecutor.registerOnTrajectoryProgressCallback(replanTriggerPercent, (pct) -> {
      if (trajectoryExecutor.getCurrentTrajectory() != null
          && trajectoryExecutor.getCurrentTrajectory().getComplexManeuver() != null) {
        OnTrajectoryProgressCallback complexReplanCallback = new OnTrajectoryProgressCallback() {
          @Override
          public void onProgress(double pct) {
            // Ensure we're only called once
            trajectoryExecutor.unregisterOnTrajectoryProgressCallback(this);

            // Trigger the replan
            arbitratorStateMachine.processEvent(ArbitratorEvent.COMPLEX_TRAJECTORY_COMPLETION_ALERT);
          }
        };

        // Schedule this to execute closer to the end of the complex trajectory
        trajectoryExecutor.registerOnTrajectoryProgressCallback(complexTrajectoryReplanTriggerPercent,
            complexReplanCallback);
      } else {
        arbitratorStateMachine.processEvent(ArbitratorEvent.TRAJECTORY_COMPLETION_ALERT);
      }
    });

    trajectoryExecutor.runTrajectory(trajectory);
    arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
  }

  /**
   * Job queue task for performing replanning after successful completion of the
   * previous trajectory
   */
  private void normalReplan() {
    log.info("Arbitrator running normal replanning process");
    if (downtrackDistance.get() < routeLength.get()) {
      increasePlanningWindow();

      Trajectory currentTrajectory = trajectoryExecutor.getCurrentTrajectory();

      double trajectoryStart = Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation());
      double trajectoryEnd = getNextTrajectoryEndpoint(trajectoryStart);

      if (Math.abs(trajectoryStart - routeLength.get()) < DISTANCE_EPSILON) {
        // Trajectory would start after route ends just don't do anything
        arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
        return;
      }

      trajectory = planTrajectory(trajectoryStart, trajectoryEnd);
      trajectoryExecutor.runTrajectory(trajectory);
      arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
    } else {
      log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
    }
  }

  /**
   * Job queue task for performing replanning after the execution of a complex maneuver
   * <p>
   * Plans temporary steadying trajectory for a configured duration then replans as usual
   */
  private void complexTrajectoryReplan() {
    log.info("Arbitrator replanning trajectory after complex maneuver execution");
    if (downtrackDistance.get() < routeLength.get()) {
      increasePlanningWindow();

      Trajectory currentTrajectory = trajectoryExecutor.getCurrentTrajectory();

      double steadyingTrajectoryStart = Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation());

      if (Math.abs(steadyingTrajectoryStart - routeLength.get()) < DISTANCE_EPSILON) {
        // Trajectory would start after route ends just don't do anything
        arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
        return;
      }

      double steadyingTrajectoryEnd = steadyingTrajectoryStart + (postComplexSteadyingDuration * currentSpeed.get());
      Trajectory steadyingTrajectory = new Trajectory(steadyingTrajectoryStart, steadyingTrajectoryEnd);

      cruisingPlugin.planTrajectory(steadyingTrajectory, currentSpeed.get());
      trajectoryExecutor.runTrajectory(steadyingTrajectory);
      trajectory = steadyingTrajectory;

      // Begin normal trajectory replanning immediately
      normalReplan();
    } else {
      log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
    }

  }

  /**
   * Job queue task for replanning after the Tracking component has notified us of
   * trajectory failure.
   */
  private void failedTrajectoryReplan() {
    log.info("Arbitrator replanning trajectory because of failed execution!");
    if (downtrackDistance.get() < routeLength.get()) {
      decreasePlanningWindow();

      double trajectoryStart = downtrackDistance.get();
      double trajectoryEnd = getNextTrajectoryEndpoint(trajectoryStart);

      trajectory = planTrajectory(trajectoryStart, trajectoryEnd);
      trajectoryExecutor.abortTrajectory();
      trajectoryExecutor.runTrajectory(trajectory);
      arbitratorStateMachine.processEvent(ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING);
    } else {
      log.warn("Arbitrator has detected route completion, but Guidance has not yet received ROUTE_COMPLETE");
    }
  }

  @Override
  public void notifyTrajectoryFailure() {
    arbitratorStateMachine.processEvent(ArbitratorEvent.TRAJECTORY_FAILED_EXECUTION);
  }

  @Override
  public void onStateChange(ArbitratorState newState) {
    switch (newState) {
    case INITIAL_PLANNING:
      jobQueue.add(this::initialPlan);
      break;
    case AWAITING_REPLAN:
      // Don't submit anything to the job queue
      break;
    case NORMAL_REPLANNING:
      jobQueue.add(this::normalReplan);
      break;
    case REPLAN_AFTER_COMPLEX_TRAJECTORY:
      jobQueue.add(this::complexTrajectoryReplan);
      break;
    case REPLAN_DUE_TO_FAILED_TRAJECTORY:
      jobQueue.add(this::failedTrajectoryReplan);
      break;
    case INIT:
      // Dont submit anything to the job queue
      break;
    default:
      throw new IllegalStateException("Unrecognized arbitrator state detected!");
    }
  }

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
      throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
    }
  }

  @Override
  public IManeuver getCurrentlyExecutingManeuver(ManeuverType maneuverType) {
    // For lack of a better place to put this (barring expanding a whole new abstraction into the PluginAPI) this is where
    // this functionality landed. Eventually this should be migrated into some kind of TrajectoryExecutor service of some
    // description. But it's needed for SpeedHarm ASAP and that system will take time to design because that's not a
    // functionality that can be exposed without some thought.

    // TODO: Move this somewhere better
    switch (maneuverType) {
    case COMPLEX:
      return trajectoryExecutor.getCurrentComplexManeuver();
    case LONGITUDINAL:
      return trajectoryExecutor.getCurrentLongitudinalManeuver();
    case LATERAL:
      return trajectoryExecutor.getCurrentLateralManeuver();
    default:
      return null;
    }
  }

  @Override
  public void requestNewPlan() {
    // TODO: Will likely need special logic at somepoint
    notifyTrajectoryFailure();
  }

  @Override
  public boolean planSubtrajectoryRecursively(Trajectory traj, double startDist, double endDist) {
    if (!(arbitratorStateMachine.getState() == ArbitratorState.NORMAL_REPLANNING)
    || !(arbitratorStateMachine.getState() == ArbitratorState.REPLAN_DUE_TO_FAILED_TRAJECTORY)) {
      // Not in valid state to plan recursively
      return false;
    }

    Trajectory tempBuffer = new Trajectory(traj); // Temporary copy to ensure we don't mess up the real one

    Trajectory plannedRegion = planTrajectory(startDist, endDist);

    // Test the insert into the copied trajectory
    for (LateralManeuver m : plannedRegion.getLateralManeuvers()) {
      if (!tempBuffer.addManeuver(m)) {
        return false;
      }
    }
    for (LongitudinalManeuver m : plannedRegion.getLongitudinalManeuvers()) {
      if (!tempBuffer.addManeuver(m)) {
        return false;
      }
    }
    if (plannedRegion.getComplexManeuver() != null)  {
      if(!tempBuffer.setComplexManeuver(plannedRegion.getComplexManeuver())) {
        return false;
      }
    }

    // We've succeeded the insert into the temporary trajectory, proceed with insert into real trajectory
    for (LateralManeuver m : plannedRegion.getLateralManeuvers()) {
      if (!traj.addManeuver(m)) {
        throw new RosRuntimeException("Recursive subtrajectory planning failed even though operations on copy were successful. Trajetory may be corrupted!");
      }
    }
    for (LongitudinalManeuver m : plannedRegion.getLongitudinalManeuvers()) {
      if (!traj.addManeuver(m)) {
        throw new RosRuntimeException("Recursive subtrajectory planning failed even though operations on copy were successful. Trajetory may be corrupted!");
      }
    }
    if (plannedRegion.getComplexManeuver() != null)  {
      if(!traj.setComplexManeuver(plannedRegion.getComplexManeuver())) {
        throw new RosRuntimeException("Recursive subtrajectory planning failed even though operations on copy were successful. Trajetory may be corrupted!");
      }
    }

    return true;
  }
}
