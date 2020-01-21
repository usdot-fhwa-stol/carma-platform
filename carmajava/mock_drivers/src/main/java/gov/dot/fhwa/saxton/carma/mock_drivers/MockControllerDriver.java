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
 * A class which can be used to simulate a controller driver for the CARMA Platform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'controller'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockControllerDriver extends AbstractMockDriver {

  // Topics
  // Published
  protected Publisher<RobotEnabled> statusPub;

  // Subscribed
  protected final Subscriber<autoware_msgs.VehicleCmd> cmdSub;

  // Services
  // Server
  protected final ServiceServer<SetEnableRoboticRequest, SetEnableRoboticResponse> enabledSrv;
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

  public MockControllerDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    statusPub = connectedNode.newPublisher("controller/robot_status", RobotEnabled._TYPE);

    // Subscribed
    cmdSub = connectedNode.newSubscriber("controller/vehicle_cmd", autoware_msgs.VehicleCmd._TYPE);

    enabledSrv = connectedNode.newServiceServer("controller/enable_robotic", cav_srvs.SetEnableRobotic._TYPE,
        new ServiceResponseBuilder<SetEnableRoboticRequest, SetEnableRoboticResponse>() {
          @Override
          public void build(SetEnableRoboticRequest arg0, SetEnableRoboticResponse arg1) throws ServiceException {
            // NO-OP
          }
        });

    // Services
    // Server
    setLightsService = connectedNode.newServiceServer("controller/set_lights", cav_srvs.SetLights._TYPE,
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
      RobotEnabled statusMsg = statusPub.newMessage();

      // Build RobotEnabled Message
      statusMsg.setBrakeDecel(Double.parseDouble(elements[BRAKE_DECEL_IDX]));
      statusMsg.setRobotEnabled(Boolean.parseBoolean(elements[ROBOT_ENABLED_IDX]));
      statusMsg.setRobotActive(Boolean.parseBoolean(elements[ROBOT_ENABLED_IDX]));
      statusMsg.setTorque(Double.parseDouble(elements[TORQUE_IDX]));

      // Publish Data
      statusPub.publish(statusMsg);
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
    return new ArrayList<>(Arrays.asList("controller"));
  }

  @Override
  public List<String> getDriverAPI() {
    return new ArrayList<>(
        Arrays.asList("controller/enable_robotic", "controller/vehicle_cmd",
            "controller/set_lights", "controller/robot_status"));
  }
}
