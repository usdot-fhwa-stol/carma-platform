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

package gov.dot.fhwa.saxton.carma.message.factory;

import java.nio.ByteOrder;
import java.util.Arrays;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.message.Time;

import cav_msgs.ByteArray;
import cav_msgs.MobilityResponse;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityHeaderHelper;
import gov.dot.fhwa.saxton.carma.message.helper.StringConverterHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility
 * Response message by using J2735 compiler shared library.
 */
public class MobilityResponseMessage implements IMessage<MobilityResponse> {

    private static int URGENCY_MAX = 1000;
    private static int URGENCY_MIN = 0;
    
    private MessageFactory factory;
    private SaxtonLogger   log;
    
    public MobilityResponseMessage(SaxtonLogger log, MessageFactory factory) {
        this.factory = factory;
        this.log     = log;
    }
    
    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
        }
    }
    
    /**
     * This is the declaration for native method. It will take data from MobilityResponse ROS message
     * and return a byte array including the encoded message. Because of the efficiency of JNI method
     * call, it takes fields directly instead of a single MobilityResponse object.
     * 
     * @return encoded MobilityResponse Message
     */
    private native byte[] encodeMobilityResponse(byte[] senderId, byte[] targetId, byte[] senderBSMId,
            byte[] planId, byte[] timestamp, int urgency, boolean isAccepted);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityResponse byte array
     * and a empty MobilityResponse object as input. It will decode the message and set all fields in MobilityResponse.
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMobilityResponse(byte[] encodedArray, Object mobilityResponse, byte[] senderId,
            byte[] targetId, byte[] bsmId, byte[] planId, byte[] timestamp);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        byte[] encodedMsg = callJniEncode((MobilityResponse) plainMessage);
        if (encodedMsg == null) {
            log.warn("MobilityResponse", "MobilityResponseMessage cannot encode the message");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binaryMsg = factory.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encodedMsg);
        binaryMsg.setContent(buffer);
        binaryMsg.setMessageType("MobilityResponse");
        binaryMsg.getHeader().setFrameId("0");
        binaryMsg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
        return new MessageContainer("ByteArray", binaryMsg);
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encodedMsg = new byte[buffer.capacity()];
        for (int i = 0; i < buffer.capacity(); i++) {
            encodedMsg[i] = buffer.getByte(i);
        }
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        // fill with character 'zero'
        Arrays.fill(timestamp, (byte) 48);
        MobilityResponse response = factory.newFromType(MobilityResponse._TYPE);
        int result = decodeMobilityResponse(encodedMsg, response, senderId, targetId, bsmId, planId, timestamp);
        if (result == -1) {
                log.warn("MobilityResponseMessage cannot decode message.");
                return new MessageContainer("MobilityResponse", null);
        }
        response.getHeader().setSenderId(StringConverterHelper.readDynamicLengthString(senderId));
        response.getHeader().setRecipientId(StringConverterHelper.readDynamicLengthString(targetId));
        response.getHeader().setSenderBsmId(new String(bsmId));
        response.getHeader().setPlanId(new String(planId));
        response.getHeader().setTimestamp(Long.parseLong(new String(timestamp)));
        return new MessageContainer("MobilityResponse", response);
    }

    public byte[] callJniEncode(MobilityResponse message) {
        MobilityHeaderHelper header = new MobilityHeaderHelper(message.getHeader());
        // we did not use a header class here, so we need to hard-code the upper bound for this value
        int urgency = Math.min(Math.max(URGENCY_MIN, message.getUrgency()), URGENCY_MAX);
        return encodeMobilityResponse(header.getSenderId(), header.getTargetId(), header.getBSMId(),
                                      header.getPlanId(), header.getTimestamp(), urgency, message.getIsAccepted());
    }
    
}
