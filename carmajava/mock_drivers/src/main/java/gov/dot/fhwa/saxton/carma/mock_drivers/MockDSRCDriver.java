/*
 * Copyright (C) 2018 LEIDOS.
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
      
      // Publish map message every 3 seconds
      message_counter++;
      if(message_counter % 30 == 0) {
          rawBytes = new byte[]{0, 18, -127, 83, 56, 3, 48, 32, 32, 75, -38, 13, 76, -36, -8, 20, 61, 77, -60, -120, 17, -122, 2, 36, 22, 72, 2, 40, 0, 8, 0, 34, -105, -44, -68, -128, -96, -96, -87, -126, 88, 37, -110, 58, -112, -78, -14, -28, 24, -104, 111, 65, -73, 0, 100, -128, 96, 36, 3, -127, 32, 32, 8, 64, 21, 72, 0, 16, 0, 69, 33, -39, -16, 1, 65, 65, 96, -57, -60, 42, 24, 121, -123, -122, 25, 80, 42, 66, -96, 96, -23, 39, 16, 6, 98, 0, 4, 0, 16, 91, -26, -65, 65, -56, -83, -19, 88, 22, -21, -64, 80, 80, 125, -53, -122, 14, -59, 122, -22, -43, 7, -98, 2, -126, -119, 0, -119, 0, 1, 0, 4, 23, 34, 58, 80, 114, -117, 117, 15, -100, 110, -87, -24, -82, 72, 10, 10, 15, 104, 116, 106, -44, 71, -64, 2, -126, -119, 0, -96, -120, 7, 4, 64, 64, 32, -128, 59, -112, 0, 32, 0, 98, -74, -115, 83, 5, -47, -7, 38, -102, 114, 80, 39, -40, 53, 47, 114, -122, 125, 108, -126, 64, 51, 64, 0, 64, 0, -59, 63, 91, 118, 26, -69, -73, -45, 93, 60, 8, 19, -20, 26, 59, -86, -63, 107, -4, 4, -128, 80, 36, 3, 1, 32, 32, 8, 64, 34, 8, 0, 16, 0, 49, 15, -27, 95, -124, -102, -51, 96, -115, -118, -50, 19, 107, 68, 0, 0, -33, -28, -128, -120, -128, 0, -128, 2, 8, 99, 101, -64, 1, 125, 22, 18, -21, 52, 2, 96, 103, 64, 72, -107, 57, 9, 7, -67, -124, -128, 80, 68, 3, 2, 32, 28, 16, 0, 36, 0, 2, 0, 0, 0, -112, 2, 97, -128, -96, -96, -14, -123, 38, 0, 20, 0, 1, 0, 0, 0, 22, -97, -63, 88, 91, -47, -38, 0, 11, 0, 0, -128, 0, 0, 10, 59, -78, -12, 57, 69, -102, -128, 6, 0, 0, 64, 0, 0, 4, 109, 85, -60, 22, -58, 127, 64};
          recvMsg.setMessageType("MAP");
      }
      
      
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
