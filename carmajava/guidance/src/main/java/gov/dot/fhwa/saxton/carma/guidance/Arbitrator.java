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

import cav_msgs.RouteState;
import com.google.common.util.concurrent.AtomicDouble;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginManager;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.OnTrajectoryProgressCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryValidationConstraint;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryValidator;
import org.apache.commons.logging.Log;
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
public class Arbitrator extends GuidanceComponent {
  protected ISubscriber<RouteState> routeStateSubscriber;
  protected ISubscriber<TwistStamped> twistSubscriber;
  protected AtomicReference<GuidanceState> state;
  protected AtomicBoolean needsReplan = new AtomicBoolean(false);
  protected PluginManager pluginManager;
  protected IPlugin lateralPlugin;
  protected IPlugin longitudinalPlugin;
  protected AtomicDouble downtrackDistance = new AtomicDouble(0.0);
  protected AtomicDouble currentSpeed = new AtomicDouble(0.0);
  protected double replanTriggerPercent = 0.75;
  protected double planningWindow = 0.0;
  protected double planningWindowGrowthFactor = 0.0;
  protected double planningWindowShrinkFactor = 0.0;
  protected int numAcceptableFailures = 0;
  protected Trajectory currentTrajectory = null;
  protected TrajectoryValidator trajectoryValidator;
  protected TrajectoryExecutor trajectoryExecutor;
  protected String lateralPluginName = "NoOp Plugin";
  protected String longitudinalPluginName = "Cruising Plugin";
  protected static final long SLEEP_DURATION_MILLIS = 100;

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
        log.error("Unable to instantiate: " + constraintClass.getCanonicalName());
        log.error(e);
      }
    }

    return constraintInstances;
  }

  @Override
  public void onGuidanceStartup() {
    log.info("Arbitrator running!");
    routeStateSubscriber = pubSubService.getSubscriberForTopic("route_status", RouteState._TYPE);
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

    // Instantiate the configured constraints and register them with the TrajectoryValidator
    List<String> constraintNames = (List<String>) ptree.getList("~trajectory_constraints");
    List<Class<? extends TrajectoryValidationConstraint>> constraintClasses = new ArrayList<>();
    for (String className : constraintNames) {
      try {
        constraintClasses.add((Class<? extends TrajectoryValidationConstraint>) Class.forName(className));
      } catch (Exception e) {
        log.warn("Unable to get Class object for name: " + className);
      }
    }

    List<TrajectoryValidationConstraint> constraints = instantiateConstraints(constraintClasses);
    for (TrajectoryValidationConstraint tvc : constraints) {
      trajectoryValidator.addValidationConstraint(tvc);
      log.info("Aribtrator using TrajectoryValidationConstraint: " + tvc.getClass().getSimpleName());
    }

    twistSubscriber = pubSubService.getSubscriberForTopic("twist_stamped", TwistStamped._TYPE);
    twistSubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
      @Override
      public void onMessage(TwistStamped msg) {
        currentSpeed.set(msg.getTwist().getLinear().getX());
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
      if (plugin.getName().equals(lateralPluginName)) {
        lateralPlugin = plugin;
      }

      if (plugin.getName().equals(longitudinalPluginName)) {
        longitudinalPlugin = plugin;
      }
    }

    if (lateralPlugin == null || longitudinalPlugin == null) {
      panic("Arbitrator unable to locate the configured and required plugins!");
    }

    log.info("Arbitrator using plugins: [" + lateralPluginName + ", " + longitudinalPluginName + "]");

    currentTrajectory = planTrajectory(downtrackDistance.get());
    trajectoryExecutor.registerOnTrajectoryProgressCallback(replanTriggerPercent, new OnTrajectoryProgressCallback() {
      @Override
      public void onProgress(double pct) {
        needsReplan.set(true);
      }
    });
    trajectoryExecutor.runTrajectory(currentTrajectory);
  }

  @Override
  public void loop() {
    if (needsReplan.get()) {
      planningWindow *= planningWindowGrowthFactor;
      planTrajectory(Math.max(downtrackDistance.get(), currentTrajectory.getEndLocation()));
      needsReplan.set(false);
    }

    try {
      Thread.sleep(SLEEP_DURATION_MILLIS);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt(); // Re-throw the exception to be handled higher up
    }
  }

  protected Trajectory planTrajectory(double trajectoryStart) {
    log.info("Arbitrator planning new trajectory spanning [" + trajectoryStart + ", " + (trajectoryStart + planningWindow)
        + ")");
    Trajectory out = null;
    for (int failures = 0; failures < numAcceptableFailures; failures++) {
      Trajectory traj = new Trajectory(trajectoryStart, trajectoryStart + planningWindow);
      double expectedEntrySpeed = 0.0;
      if (currentTrajectory != null) {
        List<IManeuver> lonManeuvers = currentTrajectory.getLongitudinalManeuvers();
        IManeuver lastManeuver = lonManeuvers.get(lonManeuvers.size() - 1);
        expectedEntrySpeed = ((LongitudinalManeuver) lastManeuver).getTargetSpeed();
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
}
