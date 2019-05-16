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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;

import java.util.List;

/**
 * Interface defining the functions needed calculating node neighbors for use with ITreeSolver
 */
public interface INeighborCalculator {

  /**
   * Provides all of the initial data necessary to identify neighbors in an EAD environment
   * @param intersections - list of known intersections, sorted in order from nearest to farthest
   * @param numIntersections - the number of intersections to consider in this solution
   * @param timeIncrement - increment between adjacent time points in the grid, sec
   * @param speedIncrement - increment between adjacent speed points in the grid, m/s
   * @param collisionChecker - Collision checker used for obstacle avoidance
   * @param planningStartTime - The time which planning is considered to have begun at. This is used for converting nodes to route locations
     @param planningStartDowntrack - The downtrack distance where planning is considered to have begun at. This is used for converting nodes to route locations
   */
  void initialize(List<IntersectionData> intersections, int numIntersections, double timeIncrement,
                  double speedIncrement, INodeCollisionChecker collisionChecker, double planningStartTime, double planningStartDowntrack);

  /**
   * Gets a list of neighbors to the provided node
   * @param node The node
   * @return List of node's neighbors
   */
  List<Node> neighbors(Node node);


  /**
   * Stores the vehicle's desired speed if traffic signals posed no constraint
   * @param os - operating speed, m/s
   */
  void setOperatingSpeed(double os);
}
