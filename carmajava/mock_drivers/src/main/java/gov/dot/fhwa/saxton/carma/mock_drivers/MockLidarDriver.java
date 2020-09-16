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
 * A class which can be used to simulate a lidar driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'lidar'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockLidarDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<sensor_msgs.PointCloud2> pointsPub;
  final Publisher<sensor_msgs.LaserScan> scanPub;

  final String pointsTopic = "lidar/points_raw";
  final String scanTopic = "lidar/scan";

  // CONSTANTS
  // TODO setup to use actual data file
  final short SAMPLE_ID_IDX = 0;
  private final short EXPECTED_DATA_COL_COUNT = 132;

  public MockLidarDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    pointsPub =
      connectedNode.newPublisher(pointsTopic, sensor_msgs.PointCloud2._TYPE);
      scanPub =
      connectedNode.newPublisher(scanTopic, sensor_msgs.LaserScan._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {
    // TODO use actual data from file
    sensor_msgs.PointCloud2 cloud = pointsPub.newMessage();
    // Set Header Data
    cloud.getHeader().setFrameId("velodyne");
    cloud.getHeader().setStamp(connectedNode.getCurrentTime());
    pointsPub.publish(cloud);

    sensor_msgs.LaserScan scan = scanPub.newMessage();
    scan.setHeader(cloud.getHeader()); // Match headers
    scanPub.publish(scan);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("lidar"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      "lidar/points_raw",
      "lidar/scan"));
  }
}
