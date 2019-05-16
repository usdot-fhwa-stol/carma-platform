/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

import java.util.List;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.ISpatialStructure;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * A ConflictDetector is responsible for providing rapid conflict detection to other guidance components
 * An internal set of vehicle paths is maintained which a suggested path can be queried against to identify conflicts
 */
public interface IConflictDetector {

  /**
   * Returns the list of conflicts between the provided path and an internally tracked set of other paths
   * 
   * @param hostPath The path representing the host trajectory
   * 
   * @return A sorted list of conflict spaces where the provided path intersects with the tracked paths
   */
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath);


  /**
   * Returns the list of conflicts between two provided vehicle paths.
   * The returned set of conflicts is only for the provided paths and ignores any other currently tracked paths
   * 
   * @param hostPath The path representing the host trajectory
   * @param otherPath The path to find conflicts with by comparing against the host trajectory
   * 
   * @return A sorted list of conflict spaces where the two paths conflict
   */
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath);

  /**
   * Returns the list of conflicts between two provided vehicle paths.
   * The returned set of conflicts is only for the provided paths and ignores any other currently tracked paths
   * 
   * @param hostPath The path representing the host trajectory
   * @param otherPath The path to find conflicts with by comparing against the host trajectory
   * @param spatialStructure The spatial structure which will store objects and provide collision checking capability
   * @param downtrackMargin The margin around a RoutePointStamped in the downtrack dimension in which a collision will be considered to have occurred
   * @param crosstrackMargin The margin around a RoutePointStamped in the crosstrack dimension in which a collision will be considered to have occurred
   * @param timeMargin The margin around a RoutePointStamped in the time dimension in which a collision will be considered to have occurred
   * @param longitudinalBias The percentage of the longitudinal conflict margin by which to bias the host vehicle's position to the front
   * @param lateralBias The percentage of the lateral conflict margin by which to bias the host vehicle's position to the right
   * @param temporalBias The percentage of the time conflict margin by which to bias the host vehicle's position forward in time
   * 
   * @return A sorted list of conflict spaces where the two paths conflict
   */
  public List<ConflictSpace> getConflicts(List<RoutePointStamped> hostPath, List<RoutePointStamped> otherPath, ISpatialStructure spatialStructure,
    double downtrackMargin, double crosstrackMargin, double timeMargin, double longitudinalBias, double lateralBias, double temporalBias);
}
