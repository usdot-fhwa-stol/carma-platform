/*
 * Copyright (C) 2018-2020 LEIDOS.
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

import java.util.List;

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
   * Function which should be called in the handleInterruptedException of a CancellableLoop
   */
  void onInterruption();

  /**
   * Returns the name of this driver
   * @return the node name
   */
  GraphName getGraphName();

  /**
   * Reads simulated data from a file and publishes it.
   * A driver data file is a csv file which can fill out all the elements of the ros messages published by a driver
   * All data files must have the first column be designated for sample id number.
   * Data rows with the same sample id number will be published during the same call to readAndPublishData
   * Other data in a data file is driver dependant and defined by the column headers and message files
   */
  void readAndPublishData();

  /**
   * Publishes the status of this driver
   */
  void publishDriverStatus();

  /**
   * Gets a list of topics names representing the api of this driver
   */
  List<String> getDriverAPI();

  /**
   * Gets the delay in ms between when data should be published from this driver
   */
  long getPublishDelay();
}
