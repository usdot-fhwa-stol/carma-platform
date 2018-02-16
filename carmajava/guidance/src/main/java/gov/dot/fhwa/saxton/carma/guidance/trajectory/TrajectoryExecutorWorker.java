/*
 * Copyright (C) 2018 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.Arbitrator;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LateralManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverRunner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package TrajectoryExecutorWorker
 * <p>
 * Performs all the non-ROS functionality of the TrajectoryWorker Guidance component.
 * Handles the execution and management of planned trajectories.
 */
public class TrajectoryExecutorWorker implements ManeuverFinishedListener {
  protected GuidanceCommands commands;
  protected List<PctCallback> callbacks = new ArrayList<>();
  protected IPublisher<cav_msgs.ActiveManeuvers> activeManeuversPub;
  protected double downtrackDistance = 0.0;
  protected AtomicReference<Trajectory> currentTrajectory = new AtomicReference<>();
  protected AtomicReference<Trajectory> nextTrajectory = new AtomicReference<>();
  protected IManeuver currentLateralManeuver = null;
  protected IManeuver currentLongitudinalManeuver = null;
  protected IManeuver currentComplexManeuver = null;
  protected double maneuverTickFrequencyHz = 10.0;
  protected ILogger log = LoggerManager.getLogger();
  cav_msgs.ActiveManeuvers activeManeuversMsg = null;
  protected Arbitrator arbitrator;
  protected int timeStepsWithoutTraj = 0;
  protected static final int MAX_ACCEPTABLE_TIMESTEPS_WITHOUT_TRAJECTORY = 3;

  // Storage struct for internal representation of callbacks based on trajectory completion percent
  private class PctCallback {
    boolean called = false;
    double pct;
    OnTrajectoryProgressCallback callback;

    PctCallback(double pct, OnTrajectoryProgressCallback callback) {
      this.pct = pct; this.callback = callback;
    }
  }

  public TrajectoryExecutorWorker(GuidanceCommands commands, double maneuverTickFrequencyHz, IPublisher<cav_msgs.ActiveManeuvers> activeManeuversPub) {
    this.commands = commands;
    this.maneuverTickFrequencyHz = maneuverTickFrequencyHz;
    this.activeManeuversPub = activeManeuversPub;
  }

  public void setArbitrator(Arbitrator arbitrator) {
    this.arbitrator = arbitrator;
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

    if (currentTrajectory == null) {
      // Nothing to do
      log.info("Finished downtrack distance update.");
      return;
    }

    // Notify subscribers if needed
    invokeCallbacks();

    log.info("Finished downtrack distance update.");
  }

  /**
   * Abort the current and queued trajectories and the currently executing maneuvers
   */
  public void abortTrajectory() {
    log.debug("TrajectoryWorker aborting currently executing trajectory.");
    currentTrajectory = null;
    nextTrajectory = null;
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
   * Get the current complex maneuver, null if none are currently executing
   */
  public IManeuver getCurrentComplexManeuver() {
    return currentComplexManeuver;
  }

  /**
   * Get the next lateral maneuver, null if none are currently executing
   */
  public IManeuver getNextLateralManeuver() {
    return currentTrajectory.get().getNextManeuverAfter(downtrackDistance, ManeuverType.LATERAL);
  }

  /**
   * Get the next longitudinal maneuver, null if none are currently executing
   */
  public IManeuver getNextLongitudinalManeuver() {
    return currentTrajectory.get().getNextManeuverAfter(downtrackDistance, ManeuverType.LONGITUDINAL);
  }

  /**
   * Get the next complex maneuver, null if none are currently executing
   */
  public IManeuver getNextComplexManeuver() {
    return currentTrajectory.get().getNextManeuverAfter(downtrackDistance, ManeuverType.COMPLEX);
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

    double numerator = downtrackDistance - currentTrajectory.get().getStartLocation();
    double denominator = currentTrajectory.get().getEndLocation() - currentTrajectory.get().getStartLocation();
    return numerator / denominator;
  }

  /**
   * Register a callback function to be invoked when the specified percent completion is acheived
   * <p>
   * Percent completion is defined over [0, 1] U (-1.0)
   */
  public synchronized void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
    callbacks.add(new PctCallback(pct, callback));
  }

  /**
   * Swap execution of the current and queued trajectory to allow to seamless handoff without entering a
   * failure mode.
   */
  private void swapTrajectories() {
    log.info("TrajectoryExecutorWorker attempting to swap to buffered Trajectory");
    // If we don't have any trajectories to swap, just exit
    if (nextTrajectory.get() == null) {
      /**
       * WARNING: Trajectories may somehow end up null due to a race condition with updateDownTrackDistance and this method
       * I've removed this nulling of current trajectory functionality as a fix, but lack complete understanding of the 
       * race condition so do not trust this code too heavily.
       */
      if (++timeStepsWithoutTraj >= MAX_ACCEPTABLE_TIMESTEPS_WITHOUT_TRAJECTORY) {
        log.warn("Timesteps without trajectory threshold exceeded, instructing arbitrator to forcibly plan!");
        if (arbitrator != null) {
          arbitrator.notifyTrajectoryFailure();
        } else {
          log.error("No Arbitrator set for replan notifications!");
        }
      }
      return;
    }

    currentTrajectory.set(nextTrajectory.get());
    nextTrajectory.set(null);

    if (currentTrajectory.get() != null) {
      log.info("TrajectoryExecutorWorker successfully swapped Trajectories");
    }

    resetCallbacks();
  }

  /**
   * Submit the specified trajectory for execution
   * </p>
   * If no trajectories are running it will be run right away, otherwise it will be queued to run after the current
   * trajectory finishes execution.
   */
  public void runTrajectory(Trajectory traj) {
    if (currentTrajectory.get() == null) {
      currentTrajectory.set(traj);

      resetCallbacks();
    } else {
      // Hold onto this trajectory for double buffering, flip to it when we finish trajectory
      nextTrajectory.set(traj);
    }

    timeStepsWithoutTraj = 0;
  }

  /**
   * Periodic loop method for iterating, this is where maneuvers get executed
   */
  public void loop() {
    if (currentTrajectory != null) {
      currentLongitudinalManeuver = currentTrajectory.get().getManeuverAt(downtrackDistance, ManeuverType.LONGITUDINAL);
      currentLateralManeuver = currentTrajectory.get().getManeuverAt(downtrackDistance, ManeuverType.LATERAL);
      currentComplexManeuver = currentTrajectory.get().getManeuverAt(downtrackDistance, ManeuverType.COMPLEX);

      if (currentComplexManeuver != null) {
        currentComplexManeuver.executeTimeStep();
      }
      if (currentLongitudinalManeuver != null) {
        currentLongitudinalManeuver.executeTimeStep();
      }
      if (currentLateralManeuver != null) {
        currentLateralManeuver.executeTimeStep();
      }
    } else {
      timeStepsWithoutTraj++;
    }
  }

  @Override
  public void onLateralManeuverFinished() {
    log.warn("Caught lateral maneuver running after its endpoint, switching maneuvers");
  }

  @Override
  public void onLongitudinalManeuverFinished() {
    log.warn("Caught longitudinal maneuver running after its endpoint, switching maneuvers");
  }

  /**
   * Get the currently executing trajectory or null if none is running
   */
  public Trajectory getCurrentTrajectory() {
    return currentTrajectory.get();
  }

  /**
   * Unregister the callback, ensuring it is no longer invoked at any point in time
   */
  public void unregisterOnTrajectoryProgressCallback(OnTrajectoryProgressCallback callback) {
    // Ensure that we don't get any weirdness when trying other operations simultaneously
    synchronized (callbacks) {
      callbacks.remove(callback);
    }
  }

  /**
   * Unregister all callbacks
   */
  public void unregisterAllTrajectoryProgressCallback() {
    // Ensure that we don't get any weirdness when trying other operations simultaneously
    synchronized (callbacks) {
      callbacks.clear();
    }
  }  
  
  /**
   * Get the current downtrack distance and then call any callbacks that have been triggered
   */
  private void invokeCallbacks() {
    // Buffer the callback list locally in case a callback modifies the callback list itself
    List<PctCallback> tmpCallbacks = new ArrayList<>();
    synchronized (callbacks) {
      tmpCallbacks.addAll(callbacks);
    }

    double completePct = getTrajectoryCompletionPct();
    for (PctCallback callback : tmpCallbacks) {
      if (!callback.called && completePct >= callback.pct) {
        log.debug("Calling Trajectory Completion callback at " + completePct);
        callback.callback.onProgress(completePct);
        callback.called = true;
      }
    }
  }

  /**
   * Force a clean restart operation on this worker componet
   */
  public void cleanRestart() {
    downtrackDistance = 0.0;
    currentTrajectory = new AtomicReference<>();
    nextTrajectory = new AtomicReference<>();
    currentLateralManeuver = null;
    currentLongitudinalManeuver = null;
    currentComplexManeuver = null;
  }

  /**
   * Reset all callbacks to as though they had not already been called
   */
  private void resetCallbacks() {
    // Take a snapshot of the contents of callbacks prior to execution
    List<PctCallback> tmpCallbacks = new ArrayList<>();
    synchronized (callbacks) {
      tmpCallbacks.addAll(callbacks);
    }

    for (PctCallback callback : tmpCallbacks) {
      callback.called = false;
    }
  }
}
