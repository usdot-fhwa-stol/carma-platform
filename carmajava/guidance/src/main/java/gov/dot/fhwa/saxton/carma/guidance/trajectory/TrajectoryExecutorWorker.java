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
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package TrajectoryExecutorWorker
 * <p>
 * Performs all the non-ROS functionality of the TrajectoryWorker Guidance component.
 * Handles the execution and management of planned trajectories.
 */
public class TrajectoryExecutorWorker {

  public TrajectoryExecutorWorker(GuidanceCommands commands) {
    this.commands = commands;
  }

  public void updateDowntrackDistance(double downtrack) {
  }

  public void abortTrajectory() {
  }

  public IManeuver getCurrentLateralManeuver() {
    return null;
  }

  public IManeuver getCurrentLongitudinalManeuver() {
    return null;
  }

  public IManeuver getNextLateralManeuver() {
    return null;
  }

  public IManeuver getNextLongitudinalManeuver() {
    return null;
  }

  public double getTrajectoryCompletionPct() {
    return -1.0;
  }

  public void registerOnTrajectoryProgressCallback(double pct, OnTrajectoryProgressCallback callback) {
  }

  public void runTrajectory(Trajectory traj) {
  }

  protected GuidanceCommands commands;
  protected List<OnTrajectoryProgressCallback> callbacks;
}
