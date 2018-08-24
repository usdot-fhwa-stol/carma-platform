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

import java.util.List;

/**
 * An interface defining the functions needed for a class which can find a path through a graph
 */
public interface ITreeSolver {
  /**
   * Function returns the path from a starting node to the goal as defined in the ICostModel
   * The neighbors of nodes on the graph are determined by the INeighborCalculator
   *
   * @param start The starting node
   * @param costModel The cost model
   * @param neighborCalculator The neighbor calculator
   * @return The path from start to goal as a list of nodes
   */
  List<Node> solve(Node start, ICostModel costModel, INeighborCalculator neighborCalculator);
}
