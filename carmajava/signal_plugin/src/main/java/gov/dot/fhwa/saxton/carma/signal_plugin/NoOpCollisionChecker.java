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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.List;

import cav_msgs.RoadwayObstacle;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * No-Op Collision Checker for the Traffic Signal Plugin. Allows NCV handling to be effectively turned off by injecting this as the primary ITrafficSignalPluginCollisionChecker
 */
public class NoOpCollisionChecker implements ITrafficSignalPluginCollisionChecker {

  @Override
  public boolean hasCollision(List<Node> trajectory, double timeOffset, double distanceOffset) {
    return false;
  }

  @Override
  public void updateObjects(List<RoadwayObstacle> obstacles) {
    // Do Nothing
  }

  @Override
  public void setHostPlan(List<Node> hostPlan, double startTime, double startDowntrack) {
    // Do Nothing
  }

}
