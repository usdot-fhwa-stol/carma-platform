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
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

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
public class MockPinPointDriver extends AbstractMockDriver {

  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  int sequenceNumber = 0;

  // Topics
  // Published
  final Publisher<cav_msgs.HeadingStamped> positionPub;
  final Publisher<sensor_msgs.NavSatFix> navSatFixPub;
  final Publisher<nav_msgs.Odometry> odometryPub;
  final Publisher<geometry_msgs.TwistStamped> velocityPub;

  // Parameters
  // Parameter	~/address	~/address	string
  // Parameter	~/port	~/port	uint16

  final int EXPECTED_DATA_ROW_COUNT = 2;

  public MockPinPointDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    positionPub = connectedNode.newPublisher("~/position/heading", cav_msgs.HeadingStamped._TYPE);
    navSatFixPub = connectedNode.newPublisher("~/position/nav_sat_fix", sensor_msgs.NavSatFix._TYPE);
    odometryPub = connectedNode.newPublisher("~/position/odometry", nav_msgs.Odometry._TYPE);
    velocityPub = connectedNode.newPublisher("~/position/velocity", geometry_msgs.TwistStamped._TYPE);
  }

  @Override public GraphName getDefaultDriverName() {
    return GraphName.of("mock_pinpoint_driver");
  }

  @Override protected void publishData(String[] data) throws IllegalArgumentException {
    if (data.length != EXPECTED_DATA_ROW_COUNT) {
      sequenceNumber++;
      throw new IllegalArgumentException(
        "Publish data called for MockAradaDriver with incorrect number of data elements. "
          + "The required number of data elements is " + EXPECTED_DATA_ROW_COUNT);
    }

    // TODO: Due to the undefined number of objects detected I will need to overload the readAndPublishData function at a minimum and maybe onStart
    // Make messages
    cav_msgs.HeadingStamped headingMsg = positionPub.newMessage();
    sensor_msgs.NavSatFix navMsg = navSatFixPub.newMessage();
    nav_msgs.Odometry odometryMsg = odometryPub.newMessage();
    geometry_msgs.TwistStamped velocityMsg = velocityPub.newMessage();

    // Set Headers
    std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
    hdr.setSeq(sequenceNumber);
    hdr.setStamp(connectedNode.getCurrentTime());

    hdr.setFrameId("pinpoint");
    headingMsg.setHeader(hdr);

    hdr.setFrameId("earth");
    navMsg.setHeader(hdr);

    hdr.setFrameId("odom");
    odometryMsg.setHeader(hdr);

    // TODO: Ask if this should be odom not base_link
    hdr.setFrameId("base_link");
    velocityMsg.setHeader(hdr);


    // Set Data
    headingMsg.setHeading(Float.parseFloat(data[0]));
    navMsg.

    // Publish Data
    recvPub.publish(recvMsg);
    sequenceNumber++;
  }

  @Override protected int getExpectedRowCount() {
    return EXPECTED_DATA_ROW_COUNT;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("comms"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(connectedNode.getName() + "/position/heading",
      connectedNode.getName() + "/position/nav_sat_fix",
      connectedNode.getName() + "/position/odometry",
      connectedNode.getName() + "/position/velocity"));
  }
}
