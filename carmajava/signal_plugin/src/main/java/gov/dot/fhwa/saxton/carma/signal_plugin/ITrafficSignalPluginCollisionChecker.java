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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.List;

import cav_msgs.RoadwayObstacle;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * The ITrafficSignalPluginCollisionChecker will provide collision checking functionality for the Traffic Signal Plugin
 * The historical descriptions of detected in lane objects are cached and used to predict future object motion
 * The object motion predictions are used for collision checking to prevent the plugin from planning a path through a detected object
 * Additionally, when an upcoming collision is detected based on new data, a total replan will be requested
 * After a replan occurs additional replans will be requested at fixed time increments
 */
public interface ITrafficSignalPluginCollisionChecker extends INodeCollisionChecker{

  @Override
  boolean hasCollision(List<Node> trajectory, double timeOffset, double distanceOffset);

  /**
   * Function to be called when new objects are detected by host vehicle sensors
   * This will update object histories and predictions
   * If a collision is detected based on new predictions then a replan will be requested
   * 
   * Note: This function assumes this data will be provided at a fixed rate which is below half the prediction period 
   * 
   * @param obstacles The list of detected objects in route space around the vehicle
   */
  void updateObjects(List<RoadwayObstacle> obstacles);

  /**
   * Sets the current host vehicle plan which will be used for collision checks
   * The provided host plan will be interpolated using the provided IMotionInterpolator
   * 
   * @param hostPlan The host plan to set as the current plan
   * @param startTime The time which planning is considered to have begun at. This is used for converting nodes to route locations
	 * @param startDowntrack The downtrack distance where planning is considered to have begun at. This is used for converting nodes to route locations
	 * 
   */
  void setHostPlan(List<Node> hostPlan, double startTime, double startDowntrack);
}
