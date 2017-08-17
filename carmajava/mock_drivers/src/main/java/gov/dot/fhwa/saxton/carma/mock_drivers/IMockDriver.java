/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.mock_drivers;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;

/**
 * Interface for Driver strategies which will be used by MockDriverNode nodes.
 * Instances of objects which implement this interface are meant to be defined in a ROS node.
 */
public interface IMockDriver {

  /**
   * Function which should be called in the onStart function of a containing ROS Node
   * @param connectedNode The node which is being started
   */
  void onStart(ConnectedNode connectedNode);

  /**
   * Function which should be called in the onShutdown function of a containing ROS Node
   * @param node The node which is being shutdown
   */
  void onShutdown(Node node);

  /**
   * Returns the default name of this driver
   * @return the default node name
   */
  GraphName getDefaultDriverName();

  /**
   * Reads simulated data from a file and publishes it
   */
  void readAndPublishData();

  /**
   * Publishes the status of this driver
   */
  void publishDriverStatus();
}
