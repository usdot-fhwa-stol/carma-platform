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

package gov.dot.fhwa.saxton.carma.guidance.util;

import org.ros.node.ConnectedNode;

/**
 * Implementation of an ITimeProvider which returns the ROS time
 */
public class ROSTimeProvider implements ITimeProvider {

  private final ConnectedNode node;
  private final long NS_PER_MS = 1000000;

  /**
   * Constructor
   * 
   * @param connectedNode The ROS node which will provide the ROS time
   */
  public ROSTimeProvider (ConnectedNode node) {
    this.node = node;
  }

  @Override
  public double getCurrentTimeSeconds() {
    return node.getCurrentTime().toSeconds();
  }

  @Override
  public long getCurrentTimeMillis() {
    return node.getCurrentTime().totalNsecs() / NS_PER_MS;
  }
}