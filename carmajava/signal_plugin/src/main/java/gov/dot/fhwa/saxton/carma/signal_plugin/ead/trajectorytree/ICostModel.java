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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree;

/**
 * Interface defining the functions needed for a cost model to be used with an ITreeSolver
 */
public interface ICostModel {
  /**
   * Calculates the cost between two nodes
   * @param n1 First node
   * @param n2 Second node
   * @return The cost
   */
  double cost(Node n1, Node n2);

  /**
   * Calculates the heuristic value of node assuming some goal.
   * A heuristic is considered admissible if it is always <= actual cost of reaching the goal
   * @param n1 The node
   * @return the heuristic value
   */
  double heuristic(Node n1);

  /**
   * Allows specification of a set of tolerances on the various Node dimensions to facilitate approximate goal comparison
   * @param tolerances A node containing allowable tolerances in each dimension
   */
  void setTolerances(Node tolerances);

  /**
   * Defines a node as the goal (primarily used for the heuristic calculation).
   * @param goal - the goal node
   */
  void setGoal(Node goal);

  /**
   * Returns true if the provided node is the equal to the goal or within specified tolerances of the goal
   * @param n The node
   * @return True if n equals goal
   *
   * @apiNote If setTolerances has not been called then the comparison must satisfy equality
   */
  boolean isGoal(Node n);

  /**
   * Returns true if n is legal but clearly not part of a practical solution for the problem at hand
   * @param n The node
   * @return true if N should be discarded from consideration
   */
  boolean isUnusable(Node n);
}
