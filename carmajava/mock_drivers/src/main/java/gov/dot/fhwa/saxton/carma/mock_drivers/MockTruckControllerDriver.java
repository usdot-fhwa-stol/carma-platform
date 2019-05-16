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

package gov.dot.fhwa.saxton.carma.mock_drivers;

import cav_msgs.LightBarStatus;
import cav_srvs.GetLightsRequest;
import cav_srvs.GetLightsResponse;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;
import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate a truck controller driver for the CarmaPlatform.
 * Currently this class is simply a duplicate of the MockSRCControllerDriver class with modified names
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'truck_controller'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockTruckControllerDriver extends AbstractMockDriver {

  // Topics
  // Published
  protected final Publisher<diagnostic_msgs.DiagnosticArray> diagnosticsPub;
  protected final Publisher<cav_msgs.RobotEnabled> enabledPub;

  // Subscribed
  protected final Subscriber<std_msgs.Float32> longEffortSub;
  protected final Subscriber<cav_msgs.SpeedAccel> subscriber;

  // Services
  // Server
  protected final ServiceServer<GetLightsRequest, GetLightsResponse> getLightsService;
  protected final ServiceServer<SetLightsRequest, SetLightsResponse> setLightsService;

  private final int EXPECTED_DATA_COL_COUNT = 9;
  private final short SAMPLE_ID_IDX = 0;
  private final short BRAKE_DECEL_IDX = 1;
  private final short ROBOT_ENABLED_IDX = 2;
  private final short TORQUE_IDX = 3;
  private final short HARDWARE_ID_IDX = 4;
  private final short DIAG_LEVEL_IDX = 5;
  private final short DIAG_MSG_IDX = 6;
  private final short DIAG_KEY_MSG = 7;
  private final short DIAG_VALUE_IDX = 8;

  // Light bar states
  protected boolean lightBarFlash = false;
  protected boolean leftArrow = false;
  protected boolean rightArrow = false;
  protected boolean takedown = false;

  public MockTruckControllerDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    diagnosticsPub =
      connectedNode.newPublisher("control/diagnostics", diagnostic_msgs.DiagnosticArray._TYPE);
    enabledPub =
      connectedNode.newPublisher("control/robot_enabled", cav_msgs.RobotEnabled._TYPE);

    // Subscribed
    longEffortSub =
      connectedNode.newSubscriber("control/cmd_longitudinal_effort", std_msgs.Float32._TYPE);
    subscriber =
      connectedNode.newSubscriber("control/cmd_speed", cav_msgs.SpeedAccel._TYPE);

    // Services
    // Server
    getLightsService = connectedNode
      .newServiceServer("control/get_lights", cav_srvs.GetLights._TYPE,
        new ServiceResponseBuilder<GetLightsRequest, GetLightsResponse>() {
          @Override public void build(GetLightsRequest request,
            GetLightsResponse response) {

            LightBarStatus lightStatus = response.getStatus();
            lightStatus.setFlash((byte)(lightBarFlash ? 1:0));
            lightStatus.setLeftArrow((byte)(leftArrow ? 1:0));
            lightStatus.setRightArrow((byte)(rightArrow ? 1:0));
            lightStatus.setTakedown((byte)(takedown ? 1:0));
            response.setStatus(lightStatus);
          }
        });
    setLightsService = connectedNode
      .newServiceServer("control/set_lights", cav_srvs.SetLights._TYPE,
        new ServiceResponseBuilder<SetLightsRequest, SetLightsResponse>() {
          @Override public void build(SetLightsRequest request,
            SetLightsResponse response) {

            LightBarStatus lightStatus = request.getSetState();
            lightBarFlash = lightStatus.getFlash() == 1;
            leftArrow = lightStatus.getLeftArrow() == 1;
            rightArrow = lightStatus.getRightArrow() == 1;
            takedown = lightStatus.getTakedown() == 1;
            log.info(
              "Lights have been set to Flash: " + lightBarFlash + " Left: " + leftArrow + " Right: "
                + rightArrow + " Takedown: " + takedown);
          }
        });
  }

  @Override protected void publishData(List<String[]> data) throws IllegalArgumentException {

    for(String[] elements : data) {
      // Make messages
      cav_msgs.RobotEnabled enabledMsg = enabledPub.newMessage();
      diagnostic_msgs.DiagnosticArray diagMsg = diagnosticsPub.newMessage();

      // Build RobotEnabled Message
      enabledMsg.setBrakeDecel(Double.parseDouble(elements[BRAKE_DECEL_IDX]));
      enabledMsg.setRobotEnabled(Boolean.parseBoolean(elements[ROBOT_ENABLED_IDX]));
      enabledMsg.setTorque(Double.parseDouble(elements[TORQUE_IDX]));

      // Build Diagnostics Message: Assumes that only diagnostic is in a data file line
      std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
      hdr.setFrameId("0");
      hdr.setSeq(Integer.parseInt(elements[SAMPLE_ID_IDX]));
      hdr.setStamp(connectedNode.getCurrentTime());

      diagMsg.setHeader(hdr);

      DiagnosticStatus diagnosticStatus = messageFactory.newFromType(DiagnosticStatus._TYPE);
      diagnosticStatus.setHardwareId(elements[HARDWARE_ID_IDX]);
      diagnosticStatus.setLevel(Byte.valueOf(elements[DIAG_LEVEL_IDX]));
      diagnosticStatus.setName(getGraphName().toString());
      diagnosticStatus.setMessage(elements[DIAG_MSG_IDX]);

      KeyValue keyValue = messageFactory.newFromType(KeyValue._TYPE);
      keyValue.setKey(elements[DIAG_KEY_MSG]);
      keyValue.setValue(elements[DIAG_VALUE_IDX]);

      diagnosticStatus.setValues(new ArrayList<>(Arrays.asList(keyValue)));
      diagMsg.setStatus(new ArrayList<>(Arrays.asList(diagnosticStatus)));

      // Publish Data
      enabledPub.publish(enabledMsg);
      diagnosticsPub.publish(diagMsg);
    }
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("lon_controller"));
  }

  @Override public List<String> getDriverAPI(){
    return new ArrayList<>(Arrays.asList(
      "control/diagnostics",
      "control/robot_enabled",
      "control/cmd_longitudinal_effort",
      "control/cmd_speed",
      "control/get_lights",
      "control/set_lights"
    ));
  }
}
