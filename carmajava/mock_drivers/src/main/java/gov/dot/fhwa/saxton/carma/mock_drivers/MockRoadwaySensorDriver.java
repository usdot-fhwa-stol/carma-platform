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
 * A class which can be used to simulate a roadway sensor driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'roadway_sensor'
 * rosparam set /mock_driver/data_file_path '/opt/carma/test_data/pinpoint_stationary.csv'
 * rosrun carma mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockRoadwaySensorDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<derived_object_msgs.LaneModels> lanesPub;
  final Publisher<derived_object_msgs.ObjectWithCovarianceArray> objectsPub;

  // CONSTANTS
  final short SAMPLE_ID_IDX = 0;
  private final short EXPECTED_DATA_COL_COUNT = 132;

  public MockRoadwaySensorDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics TODO this whole node
    // Published
    lanesPub =
      connectedNode.newPublisher("roadway_sensor/lane_models", derived_object_msgs.LaneModels._TYPE);
    objectsPub =
      connectedNode.newPublisher("roadway_sensor/detected_objects", derived_object_msgs.ObjectWithCovarianceArray._TYPE);
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {

    String frameId = "mobile_eye";
    Time currentTime = connectedNode.getCurrentTime();
    derived_object_msgs.LaneModels laneMsg = lanesPub.newMessage();
    derived_object_msgs.ObjectWithCovarianceArray objMsg = objectsPub.newMessage();

    // Build Header
    std_msgs.Header hdr = laneMsg.getHeader();
    hdr.setFrameId(frameId);
    hdr.setStamp(currentTime);

    objMsg.setHeader(hdr);

    lanesPub.publish(laneMsg);
    objectsPub.publish(objMsg);
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("roadway_sensor"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(
      "roadway_sensor/lane_models",
      "roadway_sensor/detected_objects"
    ));
  }
}
