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
 * A class which can be used to simulate a camera driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'camera'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockCameraDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<sensor_msgs.Image> imagePub;
  final Publisher<sensor_msgs.CameraInfo> infoPub;
  final String imageTopic = "camera/1/image_raw";
  final String infoTopic = "camera/1/camera_info";

  private final short EXPECTED_DATA_COL_COUNT = 132; // TODO
  private final short SAMPLE_ID_IDX = 1;

  public MockCameraDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    imagePub =
      connectedNode.newPublisher(imageTopic, sensor_msgs.Image._TYPE);
    infoPub =
      connectedNode.newPublisher(infoTopic, sensor_msgs.CameraInfo._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {
    // TODO use actual data from file
    sensor_msgs.Image image = imagePub.newMessage();
    // Set Header Data
    image.getHeader().setFrameId("camera");
    image.getHeader().setStamp(connectedNode.getCurrentTime());
    imagePub.publish(image);

    sensor_msgs.CameraInfo info = infoPub.newMessage();
    info.setHeader(image.getHeader()); // Match headers
    infoPub.publish(info);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("camera"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      imageTopic,
      infoTopic));
  }
}
