/*
 * Copyright (C) 2017 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of * the License at *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverRunner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import java.util.ArrayList;
import java.util.List;

/**
 * Guidance package TrajectoryExecutorWorker
 * <p>
 * Performs all the non-ROS functionality of the TrajectoryWorker Guidance component.
 * Handles the execution and management of planned trajectories.
 */
public class TrajectoryExecutorWorker implements ManeuverFinishedListener {
  protected GuidanceCommands commands;
  protected List<PctCallback> callbacks = new ArrayList<>();
  protected double downtrackDistance = 0.0;
  protected Trajectory currentTrajectory = null;
  protected Trajectory nextTrajectory = null;
  protected IManeuver currentLateralManeuver = null;
  protected IManeuver currentLongitudinalManeuver = null;
  protected Thread lateralManeuverThread = new Thread();
  protected Thread longitudinalManeuverThread = new Thread();
  protected double maneuverTickFrequencyHz = 10.0;
  protected ILogger log = LoggerManager.getLogger();

  // Storage struct for internal representation of callbacks based on trajectory completion percent
  private class PctCallback {
    boolean called = false;
    double pct;
    OnTrajectoryProgressCallback callback;

    PctCallback(double pct, OnTrajectoryProgressCallback callback) {
      this.pct = pct;
      this.callback = callback;
    }
  }

  public TrajectoryExecutorWorker(GuidanceCommands commands, double maneuverTickFrequencyHz) {
    this.commands = commands;
    this.maneuverTickFrequencyHz = maneuverTickFrequencyHz;
  }

  private void execute(IManeuver maneuver) {
    log.info("TrajectoryExecutorWorker running new maneuver from [" + maneuver.getStartDistance() + ", "
        + maneuver.getEndDistance() + ")");
    if (maneuver instanceof LongitudinalManeuver) {
      if (longitudinalManeuverThread != null) {
        longitudinalManeuverThread.interrupt();
      }
      ManeuverRunner runner = new ManeuverRunner(maneuver, maneuverTickFrequencyHz);
      runner.setListener(this);
      longitudinalManeuverThread = new Thread(runner);
      longitudinalManeuverThread.setName("Longitudinal Maneuver Runner");
      longitudinalManeuverThread.start();
    } else {
      if (lateralManeuverThread != null) {
        lateralManeuverThread.interrupt();
      }

      ManeuverRunner runner = new ManeuverRunner(maneuver, maneuverTickFrequencyHz);
      runner.setListener(this);
      lateralManeuverThread = new Thread(runner);
      lateralManeuverThread.setName("Lateral Maneuver Runner");
      lateralManeuverThread.start();
    }
  }

  /**
   * Check the status of the current maneuvers and start them if the need to be started
   */
  private void checkAndStartManeuvers() {
    log.debug("TrajectoryExecutorWorker checking if maneuvers need to be started.");
    // Start the maneuvers if they aren't already running
    if (currentLateralManeuver != null && downtrackDistance >= currentLateralManeuver.getStartDistance()
        && !lateralManeuverThread.isAlive()) {
      log.debug("TrajectoryExecutorWorker starting lateral maneuver");
      execute(currentLateralManeuver);
    }

    if (currentLongitudinalManeuver != null && downtrackDistance >= currentLongitudinalManeuver.getStartDistance()
        && !longitudinalManeuverThread.isAlive()) {
      log.debug("TrajectoryExecutorWorker starting longitudinal maneuver");
      execute(currentLongitudinalManeuver);
    }
  }

  private void checkAndStartNextLateralManeuver() {
    if (currentLateralManeuver != null && downtrackDistance >= currentLateralManeuver.getEndDistance()) {
      if (lateralManeuverThread != null && lateralManeuverThread.isAlive()) {
        lateralManeuverThread.interrupt();
      }
      currentLateralManeuver = currentTrajectory.getManeuverAt(currentLateralManeuver.getEndDistance(),
          ManeuverType.LATERAL);

      if (currentLateralManeuver != null && downtrackDistance >= currentLateralManeuver.getStartDistance()) {
        execute(currentLateralManeuver);
      }
    }
  }

  private void checkAndStartNextLongitudinalManeuver() {
    if (currentLongitudinalManeuver != null && downtrackDistance >= currentLongitudinalManeuver.getEndDistance()) {
      if (longitudinalManeuverThread != null && longitudinalManeuverThread.isAlive()) {
        longitudinalManeuverThread.interrupt();
      }
      currentLongitudinalManeuver = currentTrajectory.getManeuverAt(currentLongitudinalManeuver.getEndDistance(),
          ManeuverType.LONGITUDINAL);

      if (currentLongitudinalManeuver != null && downtrackDistance >= currentLongitudinalManeuver.getStartDistance()) {
        execute(currentLongitudinalManeuver);
      }
    }

  }

  /**
   * Update the TrajectoryExecutors current downtrack distance with respect to the route
   * </p>
   * Also triggers changes of maneuvers and/or trajectory and any callbacks that may be needed
   * as a result of the new trajectory completion value. This is the main event-driven method
   * for this class.
   * 
   * @param downtrack The current downtrack distance from RouteState
   */
  public void updateDowntrackDistance(double downtrack) {
    log.debug("TrajectoryExecutorWorker updating downtrack distance to " + downtrack);
    this.downtrackDistance = downtrack;

    checkAndStartManeuvers();

    // Call necessary callbacks
    double completePct = getTrajectoryCompletionPct();
    for (PctCallback callback : callbacks) {
      if (!callback.called && completePct >= callback.pct) {
        log.debug("Calling Trajectory Completion callback at " + completePct);
        callback.callback.onProgress(completePct);
        callback.called = true;
      }
    }

    checkAndStartNextLongitudinalManeuver();
    checkAndStartNextLateralManeuver();

    // If we've finished executing a maneuver, move onto the next one
    // See if we need to swap to our queued trajectory
    if (currentLateralManeuver == null && currentLongitudinalManeuver == null) {
      swapTrajectories();
      checkAndStartManeuvers();
    }
  }

  /**
   * Abort the current and queued trajectories and the currently executing maneuvers
   */
  public void abortTrajectory() {
    log.debug("TrajectoryWorker aborting currently executing trajectory.");
    currentTrajectory = null;
    nextTrajectory = null;

    if (currentLateralManeuver != null) {
      if (!lateralManeuverThread.isInterrupted()) {
        lateralManeuverThread.interrupt();
      }

      currentLateralManeuver = null;
    }

    if (currentLongitudinalManeuver != null) {
      if (!longitudinalManeuverThread.isInterrupted()) {
        longitudinalManeuverThread.interrupt();
      }

      currentLongitudinalManeuver = null;
    }
  }

  /**
   * Get the current lateral maneuver, null if none are currently executing
   */
  public IManeuver getCurrentLateralManeuver() {
    return currentLateralManeuver;
  }

  /**
   * Get the current longitudinal maneuver, null if none are currently executing
   */
  public IManeuver getCurrentLongitudinalManeuver() {
    return currentLongitudinalManeuver;
  }

  /**
   * Get the next lateral maneuver, null if none are currently executing
   */
  public IManeuver getNextLateralManeuver() {
    return currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LATERAL);
  }

  /**
   * Get the next longitudinal maneuver, null if none are currently executing
   */
  public IManeuver getNextLongitudinalManeuver() {
    return currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LONGITUDINAL);
  }

  /**
   * Get the current complection pct of the trajectory, -1.0 if a trajectory isn't currently executing
   * <p>
   * Percent completion is defined over [0, 1] U (-1.0)
   */
  public double getTrajectoryCompletionPct() {
    // Handle the case where we're not running a trajectory yet
    if (currentTrajectory == null) {
      return -1.0;
    }

    double numerator = downtrackDistance - currentTrajectory.getStartLocation();
    double denominator = currentTrajectory.getEndLocation() - currentTrajectory.getStartLocation();
    return numerator / denominator;
  }

  /**
   * Register a callback function to be invoked when the specified percent completion is acheived
   * <p>
   * Percent completion is defined over [0, 1] U (-1.0)
   */
  public void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
    callbacks.add(new PctCallback(pct, callback));
  }

  /**
   * Swap execution of the current and queued trajectory to allow to seamless handoff without entering a
   * failure mode.
   */
  private void swapTrajectories() {
    log.info("TrajectoryExecutorWorker attempting to swap to buffered Trajectory");
    // If we don't have any trajectories to swap, just exit
    if (nextTrajectory == null) {
      currentTrajectory = null;
      log.warn("TrajectoryExecutorWorker failed to swap to buffered Trajectory! It was null!");
      return;
    }

    currentTrajectory = nextTrajectory;
    nextTrajectory = null;

    if (currentTrajectory != null) {
      currentLateralManeuver = currentTrajectory.getManeuverAt(downtrackDistance, ManeuverType.LATERAL);
      currentLongitudinalManeuver = currentTrajectory.getManeuverAt(downtrackDistance, ManeuverType.LONGITUDINAL);
      log.info("TrajectoryExecutorWorker successfully swapped Trajectories");
    }

    for (PctCallback callback : callbacks) {
      callback.called = false;
    }
  }

  /**
   * Submit the specified trajectory for execution
   * </p>
   * If no trajectories are running it will be run right away, otherwise it will be queued to run after the current
   * trajectory finishes execution.
   */
  public void runTrajectory(Trajectory traj) {
    if (currentTrajectory == null) {
      this.currentTrajectory = traj;
      this.currentLateralManeuver = traj.getManeuverAt(downtrackDistance, ManeuverType.LATERAL);
      this.currentLongitudinalManeuver = traj.getManeuverAt(downtrackDistance, ManeuverType.LONGITUDINAL);
      for (PctCallback callback : callbacks) {
        callback.called = false;
      }
    } else {
      // Hold onto this trajectory for double buffering, flip to it when we finish trajectory
      nextTrajectory = traj;
    }
  }

  @Override
  public void onLateralManeuverFinished() {
    log.warn("Caught lateral maneuver running after its endpoint, switching maneuvers");
    checkAndStartNextLateralManeuver();
  }

  @Override
  public void onLongitudinalManeuverFinished() {
    log.warn("Caught longitudinal maneuver running after its endpoint, switching maneuvers");
    checkAndStartNextLongitudinalManeuver();
  }
}
