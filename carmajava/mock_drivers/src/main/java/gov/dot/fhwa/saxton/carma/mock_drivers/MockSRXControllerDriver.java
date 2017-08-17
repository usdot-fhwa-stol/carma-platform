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

import cav_msgs.ByteArray;
import cav_msgs.LightBarStatus;
import cav_srvs.GetLightsRequest;
import cav_srvs.GetLightsResponse;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;
import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Bool;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class which can be used to simulate an Arada comms driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'arada'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockSRXControllerDriver extends AbstractMockDriver {

  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  int sequenceNumber = 0;

  // Topics
  // Published
  final Publisher<diagnostic_msgs.DiagnosticArray> diagnosticsPub;
  final Publisher<cav_msgs.RobotEnabled> enabledPub;

  // Subscribed
  final Subscriber<std_msgs.Float32> longEffortSub;
  final Subscriber<cav_msgs.SpeedAccel> subscriber;

  // Services
  // Server
  final ServiceServer<GetLightsRequest, GetLightsResponse> getLightsService;
  final ServiceServer<cav_srvs.SetLightsRequest, cav_srvs.SetLightsResponse> setLightsService;

  //    Published	Parameter	~/device_port
  //    Published	Parameter	~/k_d
  //    Published	Parameter	~/k_i
  //    Published	Parameter	~/k_p

  final int EXPECTED_DATA_ROW_COUNT = 8;

  // Light bar states
  protected boolean lightBarFlash = false;
  protected boolean leftArrow = false;
  protected boolean rightArrow = false;
  protected boolean takedown = false;

  public MockSRXControllerDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    diagnosticsPub =
      connectedNode.newPublisher("~/control/diagnostics", diagnostic_msgs.DiagnosticArray._TYPE);
    enabledPub =
      connectedNode.newPublisher("~/control/robot_enabled", cav_msgs.RobotEnabled._TYPE);

    // Subscribed
    longEffortSub =
      connectedNode.newSubscriber("~/control/cmd_longitudinal_effort", std_msgs.Float32._TYPE);
    subscriber =
      connectedNode.newSubscriber("~/control/cmd_speed", cav_msgs.SpeedAccel._TYPE);

    // Services
    // Server
    getLightsService = connectedNode
      .newServiceServer("~/control/get_lights", cav_srvs.GetLights._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetLightsRequest, cav_srvs.GetLightsResponse>() {
          @Override public void build(cav_srvs.GetLightsRequest request,
            cav_srvs.GetLightsResponse response) {

            cav_msgs.LightBarStatus lightStatus = response.getStatus();
            lightStatus.setFlash((byte)(lightBarFlash?1:0));
            lightStatus.setLeftArrow((byte)(leftArrow?1:0));
            lightStatus.setRightArrow((byte)(rightArrow?1:0));
            lightStatus.setTakedown((byte)(takedown?1:0));
            response.setStatus(lightStatus);
          }
        });
    setLightsService = connectedNode
      .newServiceServer("~/control/set_lights", cav_srvs.SetLights._TYPE,
        new ServiceResponseBuilder<SetLightsRequest, SetLightsResponse>() {
          @Override public void build(cav_srvs.SetLightsRequest request,
            cav_srvs.SetLightsResponse response) {

            cav_msgs.LightBarStatus lightStatus = request.getSetState();
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

  @Override public GraphName getDefaultDriverName() {
    return GraphName.of("mock_srx_controller_driver");
  }

  @Override protected void publishData(String[] data) throws IllegalArgumentException {
    if (data.length != EXPECTED_DATA_ROW_COUNT) {
      sequenceNumber++;
      throw new IllegalArgumentException(
        "Publish data called for MockAradaDriver with incorrect number of data elements. "
          + "The required number of data elements is " + EXPECTED_DATA_ROW_COUNT);
    }

    // Make messages
    cav_msgs.RobotEnabled enabledMsg = enabledPub.newMessage();
    diagnostic_msgs.DiagnosticArray diagMsg = diagnosticsPub.newMessage();

    // Build RobotEnabled Message
    enabledMsg.setBrakeDecel(Double.parseDouble(data[0]));
    enabledMsg.setRobotEnabled(Boolean.parseBoolean(data[1]));
    enabledMsg.setTorque(Double.parseDouble(data[2]));

    // Build Diagnostics Message: Assumes that only diagnostic is in a data file line
    std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
    hdr.setFrameId("0");
    hdr.setSeq(sequenceNumber);
    hdr.setStamp(connectedNode.getCurrentTime());

    diagMsg.setHeader(hdr);

    diagnostic_msgs.DiagnosticStatus diagnosticStatus = messageFactory.newFromType(diagnostic_msgs.DiagnosticStatus._TYPE);
    diagnosticStatus.setHardwareId(data[3]);
    diagnosticStatus.setLevel(Byte.valueOf(data[4]));
    diagnosticStatus.setName("SRXController");
    diagnosticStatus.setMessage(data[5]);

    diagnostic_msgs.KeyValue keyValue = messageFactory.newFromType(diagnostic_msgs.KeyValue._TYPE);
    keyValue.setKey(data[6]);
    keyValue.setValue(data[7]);

    diagnosticStatus.setValues(new ArrayList<>(Arrays.asList(keyValue)));
    diagMsg.setStatus(new ArrayList<>(Arrays.asList(diagnosticStatus)));

    // Publish Data
    enabledPub.publish(enabledMsg);
    diagnosticsPub.publish(diagMsg);
    sequenceNumber++;
  }

  @Override protected int getExpectedRowCount() {
    return EXPECTED_DATA_ROW_COUNT;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("comms"));
  }

  @Override public List<String> getDriverAPI(){
    return new ArrayList<>(Arrays.asList(
      connectedNode.getName() + "/control/diagnostics",
      connectedNode.getName() + "/control/robot_enabled",
      connectedNode.getName() + "/control/cmd_longitudinal_effort",
      connectedNode.getName() + "/control/cmd_speed",
      connectedNode.getName() + "/control/get_lights",
      connectedNode.getName() + "/control/set_lights"
    ));
  }
}
