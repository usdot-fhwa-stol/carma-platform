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
            char[] senderId, char[] targetId, char[] planId,
            char[] timestamp, int ackType, char[] verification);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityAck byte array
     * and a empty MobilityAck object as input. It will decode the message and set all fields in MobilityAck.
     * @return
     */
    private native int decode_MobilityAck(
            byte[] encoded_array, char[] timestamp, char[] senderId, char[] targetId,
            char[] planId, Object ackType, char[] verification);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        MobilityHeaderHelper helper = new MobilityHeaderHelper(((MobilityAck) plainMessage).getHeader());
        char[] verification_input = ((MobilityAck) plainMessage).getVerificationCode().toCharArray(); 
        byte[] encode_msg = encode_MobilityAck(
                helper.getSenderId(), helper.getTargetId(), helper.getPlanId(), helper.getTimestamp(),
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
        char[] sendId = new char[36];
        char[] targetId = new char[36];
        char[] planId = new char[36];
        char[] timestamp = new char[19];
        char[] verification = new char[8];
        Arrays.fill(sendId, (char) 0);
        Arrays.fill(targetId, (char) 0);
        Arrays.fill(planId, (char) 0);
        Arrays.fill(verification, (char) 0);
        MobilityAck ackObject = messageFactory_.newFromType(MobilityAck._TYPE);
        MobilityAckType type = ackObject.getAgreement();
        int result = decode_MobilityAck(
                encoded_mobilityAck, timestamp, sendId,
                targetId, planId, type, verification);
        if(result == -1) {
            log_.warn("MobilityAck", "MobilityAckMessage cannot decode message");
            return new MessageContainer("MobilityAck", null);
        }
        ackObject.getHeader().setSenderId(new String(sendId));
        ackObject.getHeader().setRecipientId(new String(targetId));
        ackObject.getHeader().setPlanId(new String(planId));
        ackObject.getHeader().setTimestamp(Long.parseLong(new String(timestamp)));
        //TODO add verification string
        return new MessageContainer("MobilityAck", ackObject);
    }

}
