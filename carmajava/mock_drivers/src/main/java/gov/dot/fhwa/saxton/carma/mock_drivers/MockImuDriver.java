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

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate a IMU driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'pinpoint'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockImuDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<sensor_msgs.Imu> imuPub;
  final String imuTopic = "imu/raw_data";

  private final short EXPECTED_DATA_COL_COUNT = 132; // TODO
  private final short SAMPLE_ID_IDX = 1;

  public MockImuDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    imuPub =
      connectedNode.newPublisher(imuTopic, sensor_msgs.Imu._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {
    // TODO use actual data from file
    sensor_msgs.Imu imu = imuPub.newMessage();
    // Set Header Data
    imu.getHeader().setFrameId("imu");
    imu.getHeader().setStamp(connectedNode.getCurrentTime());
    imuPub.publish(imu);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("imu"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      imuTopic));
  }
}
