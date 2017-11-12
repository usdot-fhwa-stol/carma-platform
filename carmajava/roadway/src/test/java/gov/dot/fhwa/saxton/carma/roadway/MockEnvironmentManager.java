/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RoadwayEnvironment;
import cav_msgs.SystemAlert;
import geometry_msgs.TransformStamped;
import org.ros.message.Time;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.rosjava_geometry.Transform;
import tf2_msgs.TFMessage;

/**
 * ROS Agnostic implementation of IEnvironmentManager for use in unit testing the roadway package
 */
public class MockEnvironmentManager implements IEnvironmentManager {
  private final FrameTransformTree tfTree = new FrameTransformTree();
  private boolean shutdown = false;

  @Override public void publishTF(TFMessage tfMessage) {
    for (TransformStamped transform : tfMessage.getTransforms()) {
      // Add new transform to internal tree
      tfTree.update(transform);
    }
  }

  @Override public void publishRoadwayEnvironment(RoadwayEnvironment roadwayEnvMsg) {
    // Nothing to do
  }

  @Override public Transform getTransform(String parentFrame, String childFrame) {
    return tfTree.transform(childFrame, parentFrame).getTransform();
  }

  @Override public Time getTime() {
    return Time.fromMillis(System.currentTimeMillis());
  }

  @Override public void shutdown() {
    shutdown = true;
  }

  public boolean isShutdown(){
    return shutdown;
  }
}
