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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import java.util.List;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * A NodeCollisionChecker is responsible for checking for collisions between each node in a time sorted list of nodes
 */
public interface INodeCollisionChecker {

  /**
   * Checks for collisions between pairs of nodes provided in a time sorted list
   * 
   * The collision check is only performed for adjacent nodes in the list. (n0 with n1) and (n1 with n2) NOT (n0 with n2)
   * 
   * @param trajectory A time sorted list of nodes which define the host vehicle's desired trajectory
   * 
   * @return True if a collision was found between the nodes in the list. False otherwise
   */
  public boolean hasCollision(List<Node> trajectory);
}