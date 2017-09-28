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
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import javax.xml.bind.DatatypeConverter;
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
 * rosrun rosutils mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockAradaDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<cav_msgs.ByteArray> recvPub;
  final String recvTopic = "comms/inbound_binary_msg";

  // Subscribed
  Subscriber<cav_msgs.ByteArray> outboundSub;
  final String outboundTopic = "comms/outbound_binary_msg";

  private final int EXPECTED_DATA_COL_COUNT = 3;

  private final short SAMPLE_ID_IDX = 0;
  private final short MSG_TYPE_IDX = 1;
  private final short RAW_BYTES_IDX = 2;

  public MockAradaDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    recvPub = connectedNode.newPublisher("~/" + recvTopic, cav_msgs.ByteArray._TYPE);

    // Subscribed
    outboundSub = connectedNode.newSubscriber("~/" + outboundTopic, cav_msgs.ByteArray._TYPE);
    outboundSub.addMessageListener(new MessageListener<ByteArray>() {
      @Override public void onNewMessage(ByteArray byteArray) {
        log.info("Outbound " + byteArray.getMessageType() + " message received by "
          + getGraphName());
      }
    });
  }

  @Override protected void publishData(List<String[]> data) {
    for (String[] elements : data) {
      // Make messages
      cav_msgs.ByteArray recvMsg = recvPub.newMessage();

      // Set Data
      std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
      hdr.setFrameId("0");
      hdr.setSeq(Integer.parseInt(elements[SAMPLE_ID_IDX]));
      hdr.setStamp(connectedNode.getCurrentTime());

      recvMsg.setHeader(hdr);
      recvMsg.setMessageType(elements[MSG_TYPE_IDX]);

      // Raw byte data has the form "0a 1f 23"
      String rawByteString = elements[RAW_BYTES_IDX];
      // All non hex characters are removed. This does not support use of x such as 0x00
      rawByteString = rawByteString.replaceAll("[^A-Fa-f0-9]", "");
      // An uneven number of characters will have a 0 appended to the end
      if (rawByteString.length()%2 != 0)
        rawByteString = rawByteString.concat("0");
      // Convert the string to a byte array
      byte[] rawBytes = DatatypeConverter.parseHexBinary(rawByteString);
      // It seems that the ros messages byte[] is LittleEndian. Using BigEndian results in a IllegalArgumentException
      recvMsg.setContent(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, rawBytes));

      // Publish Data
      recvPub.publish(recvMsg);
    }
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx(){
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("comms"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(recvTopic, outboundTopic));
  }
}
