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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import org.apache.commons.logging.Log;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver.ManeuverType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package TrajectoryExecutorWorker
 * <p>
 * Performs all the non-ROS functionality of the TrajectoryWorker Guidance component.
 * Handles the execution and management of planned trajectories.
 */
public class TrajectoryExecutorWorker {
  protected GuidanceCommands commands;
  protected List<PctCallback> callbacks = new ArrayList<>();
  protected double downtrackDistance = 0.0;
  protected Trajectory currentTrajectory = null;
  protected Trajectory nextTrajectory = null;
  protected IManeuver currentLateralManeuver = null;
  protected IManeuver currentLongitudinalManeuver = null;
  protected boolean running = false;

  // Storage struct for internal representation of callbacks
  private class PctCallback {
    boolean called = false;
    double pct;
    OnTrajectoryProgressCallback callback;

    PctCallback(double pct, OnTrajectoryProgressCallback callback) {
      this.pct = pct;
      this.callback = callback;
    }
  }

  public TrajectoryExecutorWorker(GuidanceCommands commands) {
    this.commands = commands;
  }

  public void updateDowntrackDistance(double downtrack) {
    this.downtrackDistance = downtrack;

    // Check which maneuvers should be executing
    if (!running) {
      currentLateralManeuver = currentTrajectory.getNextManeuverAfter(0.0, ManeuverType.LATERAL);
      if (currentLateralManeuver == null) {
        throw new RuntimeException("Planned trajectory contains no lateral maneuvers!");
      }

      currentLongitudinalManeuver = currentTrajectory.getNextManeuverAfter(0.0, ManeuverType.LONGITUDINAL);
      if (currentLongitudinalManeuver == null) {
        throw new RuntimeException("Planned trajectory contains no longitudinal maneuvers!");
      }
    }

    // Start the maneuvers if they aren't already running
    if (downtrackDistance > currentLateralManeuver.getStartLocation() && !currentLateralManeuver.isRunning()) {
      currentLateralManeuver.execute();
    }
    if (downtrackDistance > currentLongitudinalManeuver.getStartLocation() && !currentLongitudinalManeuver.isRunning()) {
      currentLongitudinalManeuver.execute();
    }

    // Call necessary callbacks
    double completePct = getTrajectoryCompletionPct();
    for (PctCallback callback : callbacks) {
      if (!callback.called && completePct > callback.pct) {
        callback.callback.onProgress(completePct);
        callback.called = true;
      }
    }

    // If we've finished executing a maneuver, move onto the next one
    if (downtrackDistance >= currentLateralManeuver.getEndLocation()) {
      currentLateralManeuver.halt();
      currentLateralManeuver = currentTrajectory.getNextManeuverAfter(currentLateralManeuver.getEndLocation(), ManeuverType.LATERAL);

      if (downtrackDistance > currentLateralManeuver.getStartLocation() && !currentLateralManeuver.isRunning()) {
        currentLateralManeuver.execute();
      }
    }
    if (downtrackDistance >= currentLongitudinalManeuver.getEndLocation()) {
      currentLongitudinalManeuver.halt();
      currentLongitudinalManeuver = currentTrajectory.getNextManeuverAfter(currentLongitudinalManeuver.getEndLocation(), ManeuverType.LONGITUDINAL);

      if (downtrackDistance > currentLongitudinalManeuver.getStartLocation() && !currentLongitudinalManeuver.isRunning()) {
        currentLongitudinalManeuver.execute();
      }
    }

    // See if we need to swap to our queued trajectory
    if (currentLateralManeuver == null && currentLongitudinalManeuver == null) {
      swapTrajectories();
    }
    if (currentLateralManeuver == null || currentLongitudinalManeuver == null) {
      throw new RuntimeException("Both current and next trajectory are empty/completed. Unable to execute!");
    }
  }

  public void abortTrajectory() {
    this.currentTrajectory = null;
    this.currentLateralManeuver = null;
    this.currentLongitudinalManeuver = null;
  }

  public IManeuver getCurrentLateralManeuver() {
    return currentLateralManeuver;
  }

  public IManeuver getCurrentLongitudinalManeuver() {
    return currentLongitudinalManeuver;
  }

  public IManeuver getNextLateralManeuver() {
    return currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LATERAL);
  }

  public IManeuver getNextLongitudinalManeuver() {
    return currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LONGITUDINAL);
  }

  public double getTrajectoryCompletionPct() {
    // Handle the case where we're not running a trajectory yet
    if (currentTrajectory == null) {
      return -1.0;
    }

    return (downtrackDistance - currentTrajectory.getStartLocation())/(currentTrajectory.getEndLocation() - currentTrajectory.getStartLocation());
  }

  public void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
    callbacks.add(new PctCallback(pct, callback));
  }

  private void swapTrajectories() {
    // If we don't have any trajectories to swap, just exit
    if (nextTrajectory == null) {
      return;
    }

    currentTrajectory = nextTrajectory;
    nextTrajectory = null;
    currentLateralManeuver = currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LATERAL);
    currentLongitudinalManeuver = currentTrajectory.getNextManeuverAfter(downtrackDistance, ManeuverType.LONGITUDINAL);

     for (PctCallback callback : callbacks) {
       callback.called = false;
     }
  }

  public void runTrajectory(Trajectory traj) {
    if (currentTrajectory == null) {
      this.currentTrajectory = traj;
      this.currentLateralManeuver = traj.getNextManeuverAfter(downtrackDistance, ManeuverType.LATERAL);
      this.currentLongitudinalManeuver = traj.getNextManeuverAfter(downtrackDistance, ManeuverType.LONGITUDINAL);
      for (PctCallback callback : callbacks) {
        callback.called = false;
      }
    } else {
      // Hold onto this trajectory for double buffering, flip to it when we finish trajectory
      nextTrajectory = traj;
    }
  }
}
