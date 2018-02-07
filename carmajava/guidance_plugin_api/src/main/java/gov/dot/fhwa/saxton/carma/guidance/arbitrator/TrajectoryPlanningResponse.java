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

package gov.dot.fhwa.saxton.carma.guidance.arbitrator;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

/**
 * Response object returned by IPlugin instances when requested to plan a Trajectory
 * <p>
 * Contains the possibility to either successfully plan a Trajectory or to return one
 * ore more request codes that indicate the Arbitrator should take some action to 
 * facilitate trajectory planning for that plugin. The final discretion on these
 * requests lies in the Arbitrator.
 */
public class TrajectoryPlanningResponse {
  public enum PlanningRequest {
    REQUEST_LONGER_TRAJECTORY, REQUEST_DELAY_REPLAN, REQUEST_HIGHER_PRIORITY
  }

  protected double proposedTrajectoryEnd;
  protected long proposedReplanDelay;
  protected Set<PlanningRequest> requests = new HashSet<>();

  /**
   * Indicate that replanning should be delayed until more of the currently executing trajectory is finished executing.
   * <p>
   * Note: This request will not be respected during initial planning.
   * 
   * @param replanDelay The time to delay replanning by, in ms
   */
  public void requestDelayedReplan(long replanDelay) {
    requests.add(PlanningRequest.REQUEST_DELAY_REPLAN);
    proposedReplanDelay = replanDelay;
  }

  /**
   * Indicate that the planning should be attempted again with a new (longer) end distance.
   * 
   * @param trajectoryEnd The downtrack distance at which the trajectory should end
   */
  public void requestLongerTrajectory(double trajectoryEnd) {
    requests.add(PlanningRequest.REQUEST_LONGER_TRAJECTORY);
    proposedTrajectoryEnd = trajectoryEnd;
  }

  /**
   * Indicate that the requesting plugin needs an earlier place in the planning pipeline
   */
  public void requestHigherPriority() {
    requests.add(PlanningRequest.REQUEST_HIGHER_PRIORITY);
  }

  /**
   * Get the requests contained in this TrajectoryPlanningRequest.
   * 
   * @return The set of requests (if any exist), empty on successful planning
   */
  public Set<PlanningRequest> getRequests() {
    return requests;
  }

  /**
   * Get the proposed trajectory end distance, if one was requested by the plugin,
   * represented as an Optional value.
   * 
   * @return An Optional containing the proposed end distance if it was set, or Optional.empty() if not.
   */
  public Optional<Double> getProposedTrajectoryEnd() {
    if (requests.contains(PlanningRequest.REQUEST_LONGER_TRAJECTORY)) {
      return Optional.of(proposedTrajectoryEnd);
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get the proposed replan delay (ms), if one was requested by the plugin, represented as an Optional
   * value.
   * 
   * @return An Optional containing the proposed end distance if it was set, or Optional.empty() if not.
   */
  public Optional<Long> getProposedReplanDelay() {
    if (requests.contains(PlanningRequest.REQUEST_DELAY_REPLAN)) {
      return Optional.of(proposedReplanDelay);
    } else {
      return Optional.empty();
    }
  }

  /**
   * Get whether the plugin requested a higher planning priority.
   */
  public boolean higherPriorityRequested() {
    return requests.contains(PlanningRequest.REQUEST_HIGHER_PRIORITY);
  }

  /**
   * Get whether the planning process was successful.
   */
  public boolean wasSuccessful() {
    return requests.isEmpty();
  }
}
