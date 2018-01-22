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

package gov.dot.fhwa.saxton.carma.message.factory;

import java.nio.ByteOrder;
import java.util.Arrays;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import cav_msgs.ByteArray;
import cav_msgs.MobilityAck;
import cav_msgs.MobilityAckType;
import cav_msgs.MobilityIntro;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityHeaderHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility Ack message
 * by using J2735 compiler shared library.
 */
public class MobilityAckMessage implements IMessage<MobilityIntro>{

    protected ConnectedNode node_;
    protected SaxtonLogger log_;
    protected MessageFactory messageFactory_;
    
    public MobilityAckMessage(ConnectedNode node_, SaxtonLogger log_, MessageFactory messageFactory_) {
        super();
        this.node_ = node_;
        this.log_ = log_;
        this.messageFactory_ = messageFactory_;
    }

    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
            e.printStackTrace();
        }
    }
    
    /**
     * This is the declaration for native method. It will take data from MobilityAck message
     * object and return a byte array with encoded information.
     * 
     * @return the encoded MobilityAck message
     */
    private native byte[] encode_MobilityAck(
            int[] senderId, int[] targetId, int[] planId,
            int[] timestamp, int ackType, byte[] verification);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityAck byte array
     * and a empty MobilityAck object as input. It will decode the message and set all fields in MobilityAck.
     * @return
     */
    private native int decode_MobilityAck(
            byte[] encoded_array, byte[] senderId, byte[] targetId, byte[] planId,
            int[] timestamp, Object ackType, byte[] verification);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        MobilityHeaderHelper helper = new MobilityHeaderHelper(((MobilityAck) plainMessage).getHeader());
        String verification = ((MobilityAck) plainMessage).getVerificationCode();
        byte[] verification_input = new byte[verification.length()];
        for(int i = 0; i < verification.length(); i++) {
            verification_input[i] = (byte) verification.charAt(i);
        }
        byte[] encode_msg = encode_MobilityAck(
                helper.getSenderId(), helper.getTargetId(), helper.getPlanId(), helper.getTimestamp().getTimestamp(),
                ((MobilityAck) plainMessage).getAgreement().getType(), verification_input);
        if(encode_msg == null) {
            log_.warn("MobilityAck", "MobilityAckMessage cannot encode message.");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binary_msg = messageFactory_.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
        binary_msg.setContent(buffer);
        binary_msg.setMessageType("MobilityAck");
        binary_msg.getHeader().setFrameId("0");
        binary_msg.getHeader().setStamp(node_.getCurrentTime());
        return new MessageContainer("MobilityAck", binary_msg);
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encoded_mobilityAck = new byte[buffer.capacity()];
        for(int i = 0; i < buffer.capacity(); i++) {
            encoded_mobilityAck[i] = buffer.getByte(i);
        }
        byte[] sendId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] planId = new byte[16];
        int[] creationDateTime = new int[7];
        byte[] verification = new byte[8];
        Arrays.fill(sendId, (byte) 0);
        Arrays.fill(targetId, (byte) 0);
        Arrays.fill(planId, (byte) 0);
        Arrays.fill(creationDateTime, 0);
        Arrays.fill(verification, (byte) 0);
        MobilityAck ackObject = messageFactory_.newFromType(MobilityAck._TYPE);
        MobilityAckType type = ackObject.getAgreement();
        int result = decode_MobilityAck(
                encoded_mobilityAck, sendId, targetId, planId,
                creationDateTime, type, verification);
        if(result == -1) {
            log_.warn("MobilityAck", "MobilityAckMessage cannot decode message");
            return new MessageContainer("MobilityAck", null);
        }
        ChannelBuffer senderIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, sendId);
        ackObject.getHeader().setSenderId(senderIdBuffer);
        ChannelBuffer targetIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, targetId);
        ackObject.getHeader().setRecipientId(targetIdBuffer);
        ChannelBuffer planIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, planId);
        ackObject.getHeader().setPlanId(planIdBuffer);
        ackObject.getHeader().getTimestamp().setYear((short) creationDateTime[0]);
        ackObject.getHeader().getTimestamp().setMonth((byte) creationDateTime[1]);
        ackObject.getHeader().getTimestamp().setDay((byte) creationDateTime[2]);
        ackObject.getHeader().getTimestamp().setHour((byte) creationDateTime[3]);
        ackObject.getHeader().getTimestamp().setMinute((byte) creationDateTime[4]);
        ackObject.getHeader().getTimestamp().setSecond(creationDateTime[5]);
        ackObject.getHeader().getTimestamp().setOffset((short) creationDateTime[6]);
        StringBuffer verificationString = new StringBuffer();
        for(byte ch : verification) {
            if(ch == 0) {
                break;
            }
            verificationString.append((char) ch);
        }
        ackObject.setVerificationCode(verificationString.toString());
        return new MessageContainer("MobilityAck", ackObject);
    }

}
