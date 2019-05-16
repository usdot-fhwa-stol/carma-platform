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

import cav_srvs.GetLightsRequest;
import cav_srvs.GetLightsResponse;
import cav_srvs.SetEnableRoboticRequest;
import cav_srvs.SetEnableRoboticResponse;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;
import org.ros.exception.ServiceException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import cav_msgs.RobotEnabled;
import cav_msgs.LightBarStatus; //MF 02/2019 Added Light Bar Status Topic

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate an SRX controller driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'srx_controller'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockSRXControllerDriver extends AbstractMockDriver {

  // Topics
  // Published
  protected final Publisher<diagnostic_msgs.DiagnosticArray> diagnosticsPub;
  protected Publisher<RobotEnabled> statusPub;
  protected Publisher<LightBarStatus> lightBarStatusPub; //MF 02/2019

  // Subscribed
  protected final Subscriber<std_msgs.Float32> longEffortSub;
  protected final Subscriber<cav_msgs.SpeedAccel> subscriber;

  // Services
  // Server
  protected final ServiceServer<SetEnableRoboticRequest, SetEnableRoboticResponse> enabledSrv;
  protected final ServiceServer<GetLightsRequest, GetLightsResponse> getLightsService;
  protected final ServiceServer<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse> setLightsService;

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

  public MockSRXControllerDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    statusPub = connectedNode.newPublisher("control/robot_status", RobotEnabled._TYPE);
    lightBarStatusPub = connectedNode.newPublisher("control/light_bar_status", LightBarStatus._TYPE); //MF 02/2019

    diagnosticsPub = connectedNode.newPublisher("/diagnostics", diagnostic_msgs.DiagnosticArray._TYPE);
    enabledSrv = connectedNode.newServiceServer("control/enable_robotic", cav_srvs.SetEnableRobotic._TYPE,
        new ServiceResponseBuilder<SetEnableRoboticRequest, SetEnableRoboticResponse>() {
          @Override
          public void build(SetEnableRoboticRequest arg0, SetEnableRoboticResponse arg1) throws ServiceException {
            // NO-OP
          }
        });

    // Subscribed
    longEffortSub = connectedNode.newSubscriber("control/cmd_longitudinal_effort", std_msgs.Float32._TYPE);
    subscriber = connectedNode.newSubscriber("control/cmd_speed", cav_msgs.SpeedAccel._TYPE);

    // Services
    // Server
    getLightsService = connectedNode.newServiceServer("control/get_lights", cav_srvs.GetLights._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse>() {
          @Override
          public void build(cav_srvs.GetLightsRequest request, cav_srvs.GetLightsResponse response) {

            cav_msgs.LightBarStatus lightStatus = response.getStatus();
            lightStatus.setFlash((byte) (lightBarFlash ? 1 : 0));
            lightStatus.setLeftArrow((byte) (leftArrow ? 1 : 0));
            lightStatus.setRightArrow((byte) (rightArrow ? 1 : 0));
            lightStatus.setTakedown((byte) (takedown ? 1 : 0));
            response.setStatus(lightStatus);
          }
        });
    setLightsService = connectedNode.newServiceServer("control/set_lights", cav_srvs.SetLights._TYPE,
        new ServiceResponseBuilder<SetLightsRequest, SetLightsResponse>() {
          @Override
          public void build(cav_srvs.SetLightsRequest request, cav_srvs.SetLightsResponse response) {

            cav_msgs.LightBarStatus lightStatus = request.getSetState();
            lightBarFlash = lightStatus.getFlash() == 1;
            leftArrow = lightStatus.getLeftArrow() == 1;
            rightArrow = lightStatus.getRightArrow() == 1;
            takedown = lightStatus.getTakedown() == 1;
            log.info("Lights have been set to Flash: " + lightBarFlash + " Left: " + leftArrow + " Right: " + rightArrow
                + " Takedown: " + takedown);
          }
        });
  }
  
  @Override
  protected void publishData(List<String[]> data) throws IllegalArgumentException {

    for (String[] elements : data) {
      // Make messages
      diagnostic_msgs.DiagnosticArray diagMsg = diagnosticsPub.newMessage();
      RobotEnabled statusMsg = statusPub.newMessage();
      LightBarStatus lightBarStatusMsg = lightBarStatusPub.newMessage(); //MF 02/2019

      // //MF 02/2019: Build Light Bar Status Message //Right Arrow
      // GREEN FLASH
      lightBarStatusMsg.setGreenFlash(LightBarStatus.ON);
      lightBarStatusMsg.setLeftArrow(LightBarStatus.OFF);
      lightBarStatusMsg.setRightArrow(LightBarStatus.OFF);
      lightBarStatusMsg.setGreenSolid(LightBarStatus.OFF);
      lightBarStatusMsg.setFlash(LightBarStatus.OFF);
      lightBarStatusMsg.setTakedown(LightBarStatus.OFF);

      // Build RobotEnabled Message
      statusMsg.setBrakeDecel(Double.parseDouble(elements[BRAKE_DECEL_IDX]));
      statusMsg.setRobotEnabled(Boolean.parseBoolean(elements[ROBOT_ENABLED_IDX]));
      statusMsg.setRobotActive(Boolean.parseBoolean(elements[ROBOT_ENABLED_IDX]));
      statusMsg.setTorque(Double.parseDouble(elements[TORQUE_IDX]));

      // Build Diagnostics Message: Assumes that only diagnostic is in a data file line
      std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
      hdr.setFrameId("0");
      hdr.setSeq(Integer.parseInt(elements[SAMPLE_ID_IDX]));
      hdr.setStamp(connectedNode.getCurrentTime());

      diagMsg.setHeader(hdr);

      diagnostic_msgs.DiagnosticStatus diagnosticStatus = messageFactory
          .newFromType(diagnostic_msgs.DiagnosticStatus._TYPE);
      diagnosticStatus.setHardwareId(elements[HARDWARE_ID_IDX]);
      diagnosticStatus.setLevel(Byte.valueOf(elements[DIAG_LEVEL_IDX]));
      diagnosticStatus.setName(getGraphName().toString());
      diagnosticStatus.setMessage(elements[DIAG_MSG_IDX]);

      diagnostic_msgs.KeyValue keyValue = messageFactory.newFromType(diagnostic_msgs.KeyValue._TYPE);
      keyValue.setKey(elements[DIAG_KEY_MSG]);
      keyValue.setValue(elements[DIAG_VALUE_IDX]);

      diagnosticStatus.setValues(new ArrayList<>(Arrays.asList(keyValue)));
      diagMsg.setStatus(new ArrayList<>(Arrays.asList(diagnosticStatus)));

      // Publish Data
      diagnosticsPub.publish(diagMsg);
      statusPub.publish(statusMsg);
      lightBarStatusPub.publish(lightBarStatusMsg); //MF 02/2019
    }
  }

  @Override
  protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override
  protected short getSampleIdIdx() {
    return SAMPLE_ID_IDX;
  }

  @Override
  protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("lon_controller"));
  }

  @Override
  public List<String> getDriverAPI() {
    return new ArrayList<>(
        Arrays.asList("control/enable_robotic", "control/cmd_longitudinal_effort",
            "control/cmd_speed", "control/get_lights", "control/set_lights", "control/robot_status"));
  }
}
