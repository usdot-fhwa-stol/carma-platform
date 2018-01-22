/*
 * Copyright (C) 2017 LEIDOS.
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
import cav_srvs.SendMessageRequest;
import cav_srvs.SendMessageResponse;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import javax.xml.bind.DatatypeConverter;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

/**
 * A class which can be used to simulate an Arada comms driver for the CarmaPlatform.
 * <p>
 * Command line test:
 * ROSJava does not support rosrun parameter setting so a rosrun is a multi step process
 * rosparam set /mock_driver/simulated_driver 'arada'
 * rosparam set /mock_driver/data_file_path '/home/username/temp.csv'
 * rosrun carmajava mock_drivers gov.dot.fhwa.saxton.carma.mock_drivers.MockDriverNode
 */
public class MockDSRCDriver extends AbstractMockDriver {

  // Topics
  // Published
  final Publisher<cav_msgs.ByteArray> recvPub;
  final String recvTopic = "comms/inbound_binary_msg";

  // Subscribed
  Subscriber<cav_msgs.ByteArray> outboundSub;
  final String outboundTopic = "comms/outbound_binary_msg";

  //Services
  protected ServiceServer<cav_srvs.SendMessageRequest, cav_srvs.SendMessageResponse> sendServer;
  final String sendService = "comms/send";

  private final int EXPECTED_DATA_COL_COUNT = 3;

  private final short SAMPLE_ID_IDX = 0;
  private final short MSG_TYPE_IDX = 1;
  private final short RAW_BYTES_IDX = 2;
  
  long pause_length = 4000; // Set one bsm to pause for 4 seconds and resume for 4 seconds...
  int current_vehicle = 0; // Do not change it.
  int pulishDelay = 1000; // Set to 1 second length between each BSM from the same vehicle
  int vehicle_number = 3; //Need to match the length of binary data array
  int message_counter = 0; // Let driver send different inbound binary bytes

  public MockDSRCDriver(ConnectedNode connectedNode) {
    super(connectedNode);
    // Topics
    // Published
    recvPub = connectedNode.newPublisher("~/" + recvTopic, cav_msgs.ByteArray._TYPE);

    // Subscribed
    outboundSub = connectedNode.newSubscriber("~/" + outboundTopic, cav_msgs.ByteArray._TYPE);
    outboundSub.addMessageListener(new MessageListener<ByteArray>() {
      @Override public void onNewMessage(ByteArray byteArray) {
        log.debug("Outbound " + byteArray.getMessageType() + " message received by " + getGraphName());
      }
    });

    //Services
    //Server
    sendServer = connectedNode.newServiceServer("~/" + sendService, cav_srvs.SendMessage._TYPE,
      new ServiceResponseBuilder<SendMessageRequest, SendMessageResponse>() {
        @Override public void build(SendMessageRequest sendMessageRequest,
          SendMessageResponse sendMessageResponse) throws ServiceException {
          log.info("Send request received by " + getGraphName() + " with contents " + sendMessageRequest);
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
      // String rawByteString = elements[RAW_BYTES_IDX];
      // Set to static data for test
      
      String[] rawByteString = {
    		  "00 14 25 03 97 0d 6b 3b 13 39 26 6e 92 6a 1e a6 c1 55 90 00 7f ff 8c cc af ff f0 80 7e fa 1f a1 00 7f ff 08 00 4b 09 b0",
    		  "00 14 25 03 fa 2f 24 8e 1c 51 a6 6e 8c 2a 1e a6 bd 3b 90 00 7f ff 8c cc af ff f0 80 7e fa 1f a1 00 7f ff 08 00 4b 09 b0",
    		  "00 14 25 18 ae 7d a9 0e 48 81 e6 6e 95 58 1e a6 cb e1 90 00 7f ff 8c cc af ff f0 80 7e fa 1f a1 00 7f ff 08 00 4b 09 b0"
      };
      
      String currentByteString = rawByteString[current_vehicle++ % rawByteString.length];
      
      //publish mobility intro message some time
      message_counter++; 
      if(message_counter % 3 == 0) {
          recvMsg.setMessageType("MobilityIntro");
          currentByteString = "00 f0 80 a7 00 40 40 40 40 40 40 40 40 40 40 40 40 40 40 40 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 bf bf 10 84 00 00 00 21 c0 a2 ad d4 cb cf a2 0a 5b f0 e4 ba 00 10 08 02 84 03 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 f7 e2 60 80 00 00 04 38 56 dc 3c 3c b6 e1 a1 b3 0f 4c db f9 6d 41 d9 92 e6 4b 99 dd";
      } else if(message_counter % 3 == 1) {
          recvMsg.setMessageType("MobilityAck");
          currentByteString = "00 f2 3a 00 40 40 40 40 40 40 40 40 40 40 40 40 40 40 40 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 bf bf 10 84 00 00 00 21 c1 00";
      }
      
      
      boolean publish_control = false;
      if(currentByteString.equals("00 14 25 03 97 0d 6b 3b 13 39 26 6e 92 6a 1e a6 c1 55 90 00 7f ff 8c cc af ff f0 80 7e fa 1f a1 00 7f ff 08 00 4b 09 b0")) {
    	  publish_control = true;
      }

      // All non hex characters are removed. This does not support use of x such as 0x00
      currentByteString = currentByteString.replaceAll("[^A-Fa-f0-9]", "");
      
      // An uneven number of characters will have a 0 appended to the end
      if (currentByteString.length() % 2 != 0) {
    	  currentByteString = currentByteString.concat("0");
      }
      
      // Convert the string to a byte array
      byte[] rawBytes = DatatypeConverter.parseHexBinary(currentByteString);
      
      // It seems that the ros messages byte[] is LittleEndian. Using BigEndian results in a IllegalArgumentException
      recvMsg.setContent(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, rawBytes));
      
      // Publish Data
      if(!publish_control || (publish_control && ((System.currentTimeMillis() % (pause_length * 2)) < pause_length))) {
    	  recvPub.publish(recvMsg);
      }
    }
  }

  @Override protected short getExpectedColCount() {
    return EXPECTED_DATA_COL_COUNT;
  }

  @Override protected short getSampleIdIdx() {
    return SAMPLE_ID_IDX;
  }

  @Override protected List<String> getDriverTypesList() {
    return new ArrayList<>(Arrays.asList("comms"));
  }

  @Override public List<String> getDriverAPI() {
    return new ArrayList<>(Arrays.asList(recvTopic, outboundTopic, sendService));
  }
  
  @Override public long getPublishDelay() {
	  
	  return pulishDelay / vehicle_number; //Set delay here
  }
}
