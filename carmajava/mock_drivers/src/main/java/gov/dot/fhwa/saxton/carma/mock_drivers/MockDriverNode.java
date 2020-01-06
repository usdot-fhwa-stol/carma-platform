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

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;

/**
 * A class which can be used to mimick different drivers for the CarmaPlatform.
 * For specific types of drivers this class should be extended.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a two step process
 * rosparam set /mock_driver/simulated_driver 'can'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockDriverNode extends SaxtonBaseNode {
  private String defaultName =  "mock_driver";

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of(defaultName);
  }

  @Override public void onSaxtonStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    final ParameterTree params = connectedNode.getParameterTree();
    final IMockDriver simulatedDriver;
    switch (params.getString("~/simulated_driver")) {
      case "can":
        simulatedDriver = new MockCANDriver(connectedNode);
        break;
      case "comms":
        simulatedDriver = new MockCommsDriver(connectedNode);
        break;
      case "controller":
        simulatedDriver = new MockControllerDriver(connectedNode);
        break;
      case "radar":
        simulatedDriver = new MockRadarDriver(connectedNode);
        break;
      case "imu":
        simulatedDriver = new MockImuDriver(connectedNode);
        break;
      case "gnss":
        simulatedDriver = new MockGnssDriver(connectedNode);
        break;
      case "lidar":
        simulatedDriver = new MockLidarDriver(connectedNode);
        break;
      case "roadway_sensor":
        simulatedDriver = new MockRoadwaySensorDriver(connectedNode);
        break;
      case "camera":
        simulatedDriver = new MockCameraDriver(connectedNode);
        break;
      default:
        log.warn(
          "No valid driver name specified on the simulated_driver parameter. Defaulting to CAN driver");
        simulatedDriver = new MockCANDriver(connectedNode);
        break;
    }

    simulatedDriver.onStart(connectedNode);
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {

      @Override protected void loop() throws InterruptedException {
        simulatedDriver.publishDriverStatus();
        simulatedDriver.readAndPublishData();

        Thread.sleep(simulatedDriver.getPublishDelay());
      }//loop

      @Override protected void handleInterruptedException(InterruptedException e) {
        simulatedDriver.onInterruption();
        super.handleInterruptedException(e);
      }
    });//executeCancellableLoop

  }//onStart

  @Override protected void handleException(Throwable e) {

  }
}//SaxtonBaseNode
