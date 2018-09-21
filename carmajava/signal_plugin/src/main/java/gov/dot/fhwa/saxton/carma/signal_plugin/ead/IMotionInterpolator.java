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

import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * A MotionInterpolator is responsible for interpolating the position of the host vehicle between two or more nodes in a plan
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 * 
 */
public interface IMotionInterpolator {

  /**
   * Interpolates the vehicle position between each pair of nodes in a vehicle's path.
   * The returned list is in the RoutePointStamped format for use with conflict detection systems provided by CARMA
   * NOTE: The lane, crosstrack, and route segment values in this list will not be set
   * NOTE: Duplicate nodes are not allowed to be present
   * TODO: Should the crosstrack be set or always 0?
   * The returned list will always include all points in the provided trajectory
   * 
   * @param path A time sorted list of nodes which define the host vehicle's desired trajectory
   * @param distanceStep The max distance gap between each point in the output list
   * 
   * @return A list of route point stamped. The lanes and route segment values will not be set.
   */
  public List<RoutePointStamped> interpolateMotion(List<Node> trajectory, double distanceStep);
}
