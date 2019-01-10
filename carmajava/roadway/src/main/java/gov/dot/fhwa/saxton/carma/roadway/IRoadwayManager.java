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

package gov.dot.fhwa.saxton.carma.roadway;

import org.ros.message.Time;
import org.ros.rosjava_geometry.Transform;
import tf2_msgs.TFMessage;

/**
 * Interface defines the needed functions of a route manager
 */
public interface IRoadwayManager {

  /**
   * Publishes a tf2 transform ros message
   */
  void publishTF(TFMessage tfMessage);

  /**
   * Publishes a roadway environment ros message
   */
  void publishRoadwayEnvironment(cav_msgs.RoadwayEnvironment roadwayEnvMsg);

  /**
   * Gets the transform of between the requested frames
   */
  Transform getTransform(String parentFrame, String childFrame, Time stamp);

  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();

  /**
   * Safely shutdown the node
   */
  void shutdown();

}
