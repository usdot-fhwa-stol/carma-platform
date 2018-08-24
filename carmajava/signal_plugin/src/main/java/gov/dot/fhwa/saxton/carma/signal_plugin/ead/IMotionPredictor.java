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
import java.util.Queue;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * A MotionInterpolator is responsible for interpolating the position of the host vehicle between two or more nodes in a plan
 * The returned list of RoutePointStamped objects can be used to check conflicts with the CARMA conflict detection system
 */
public interface IMotionPredictor {

  /**
   * TODO use of queue vs list in description
   * Predicts the continued motion of a vehicle based on the historical trajectory
   * The result is a set of points describing the anticipated motion of the vehicle along the route
   * 
   * NOTE: Currently it is assumed position the input and output data are described in the same frame
   * 
   * @param objId A unique id which corresponds to the specific object the provided historical trajectory describes. 
   *           The id can be used for multi-object and or state-full implementations.
   * @param objTrajectory A time sorted list of nodes describing the historical behavior of the vehicle
   * @param distanceStep The max distance gap by which points in the output list will be separated
   * @param timeDuration The amount of time to project motion forward for
   * 
   * @return A list of RoutePointStamped which can be plugged into the conflict detection system provided by CARMA. The returned list is exclusive of the history data.
   */

  public List<RoutePointStamped> predictMotion(String objId, Queue<RoadwayObstacle> objTrajectory, double distanceStep, double timeDuration);
}