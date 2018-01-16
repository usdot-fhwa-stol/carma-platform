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
import cav_msgs.MobilityIntro;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityIntroductionMessageHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility Introduction message
 * by using J2735 compiler shared library.
 */
public class MobilityIntroductionMessage implements IMessage<MobilityIntro>{

    protected ConnectedNode node_;
    protected SaxtonLogger log_;
    protected MessageFactory messageFactory_;
    
    public MobilityIntroductionMessage(ConnectedNode node, SaxtonLogger log, MessageFactory messageFactory) {
        this.node_ = node;
        this.log_ = log;
        this.messageFactory_ = messageFactory;
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
     * This is the declaration for native method. It will take data from MobilityIntro message
     * object and return an byte array with encoded information. Because of the
     * efficiency of JNI method call, it takes fields directly instead of a single MobilityIntro object.
     * 
     * @return encoded Mobility Intro Message 
     */
    private native byte[] encode_MobilityIntro(
            int[] senderId, int[] targetId, int[] planId, int[] timestamp, int vehicleType,
            int[] roadwayId, int position, int laneId, int speed, int planType,
            int planParam, int[] publicKey, int[] expiration, int[] capabilities);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityIntro byte array
     * and a empty Mobility object as input. It will decode the message and set all fields in MobilityIntro.
     * Because of the efficiency of JNI method call, it takes different parts of MobilityIntro as parameters
     * instead of a single MobilityIntro object.
     * @return -1 means decode failed; 0 means decode is successful
     */
    private native int decode_MobilityIntro(
            byte[] encoded_array, Object plain_msg, byte[] senderId, byte[] targetId,
            byte[] planId, int[] dateTime, Object vehicleType, byte[] roadId,
            Object planType, byte[] publicKey, int[] expiration, byte[] capabilities);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        MobilityIntroductionMessageHelper helper = new MobilityIntroductionMessageHelper((MobilityIntro) plainMessage);
        byte[] encode_msg = encode_MobilityIntro(
                helper.getHeaderHelper().getSenderId(), helper.getHeaderHelper().getTargetId(),
                helper.getHeaderHelper().getPlanId(), helper.getHeaderHelper().getTimestamp().getTimestamp(),
                helper.getVehicleType(), helper.getRoadwayId(), helper.getPosition(), helper.getLaneId(),
                helper.getSpeed(), helper.getPlanType(), helper.getPlanParam(), helper.getPublicKey(),
                helper.getExpiration().getTimestamp(), helper.getCapabilities()
                );
        if(encode_msg == null) {
            log_.info("MobilityIntro", "MobilityIntroMessage cannot encode message.");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binary_msg = messageFactory_.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
        binary_msg.setContent(buffer);
        binary_msg.setMessageType("MobilityIntro");
        binary_msg.getHeader().setFrameId("MessageConsumer");
        binary_msg.getHeader().setStamp(node_.getCurrentTime());
        return new MessageContainer("MobilityIntro", binary_msg);
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer channelBuffer = binaryMessage.getContent();
        byte[] encoded_mobilityIntro = new byte[channelBuffer.capacity()];
        for(int i = 0; i < channelBuffer.capacity(); i++) {
            encoded_mobilityIntro[i] = channelBuffer.getByte(i);
        }
        byte[] sendId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] planId = new byte[16];
        int[] creationDateTime = new int[7];
        byte[] roadwayId = new byte[50];
        byte[] publicKey = new byte[64];
        int[] expirationDateTime = new int[7];
        byte[] capabilities = new byte[100];
        Arrays.fill(sendId, (byte) 0);
        Arrays.fill(targetId, (byte) 0);
        Arrays.fill(planId, (byte) 0);
        Arrays.fill(creationDateTime, 0);
        Arrays.fill(roadwayId, (byte) 0);
        Arrays.fill(publicKey, (byte) 0);
        Arrays.fill(expirationDateTime, 0);
        Arrays.fill(capabilities, (byte) 0);
        MobilityIntro introObject = messageFactory_.newFromType(MobilityIntro._TYPE);
        int result = decode_MobilityIntro(
                encoded_mobilityIntro, introObject, sendId, targetId, planId,
                creationDateTime, introObject.getMyEntityType(), roadwayId,
                introObject.getPlanType(), publicKey, expirationDateTime, capabilities);
        if(result == -1) {
            log_.error("MobilityIntro", "MobilityIntroMessage cannot decode message");
            return new MessageContainer("MobilityIntro", null);
        }
        ChannelBuffer senderIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, sendId);
        introObject.getHeader().setSenderId(senderIdBuffer);
        ChannelBuffer targetIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, targetId);
        introObject.getHeader().setRecipientId(targetIdBuffer);
        ChannelBuffer planIdBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, planId);
        introObject.getHeader().setPlanId(planIdBuffer);
        introObject.getHeader().getTimestamp().setYear((short) creationDateTime[0]);
        introObject.getHeader().getTimestamp().setMonth((byte) creationDateTime[1]);
        introObject.getHeader().getTimestamp().setDay((byte) creationDateTime[2]);
        introObject.getHeader().getTimestamp().setHour((byte) creationDateTime[3]);
        introObject.getHeader().getTimestamp().setMinute((byte) creationDateTime[4]);
        introObject.getHeader().getTimestamp().setSecond(creationDateTime[5]);
        introObject.getHeader().getTimestamp().setOffset((short) creationDateTime[6]);
        StringBuffer roadwayIdBuffer = new StringBuffer();
        for(byte ch : roadwayId) {
            if(ch >= 32 && ch < 127) {
                roadwayIdBuffer.append((char) ch);
            }
        }
        introObject.setMyRoadwayLink(roadwayIdBuffer.toString());
        ChannelBuffer keyBuffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, publicKey);
        introObject.setMyPublicKey(keyBuffer);
        introObject.getExpiration().setYear((short) expirationDateTime[0]);
        introObject.getExpiration().setMonth((byte) expirationDateTime[1]);
        introObject.getExpiration().setDay((byte) expirationDateTime[2]);
        introObject.getExpiration().setHour((byte) expirationDateTime[3]);
        introObject.getExpiration().setMinute((byte) expirationDateTime[4]);
        introObject.getExpiration().setSecond(expirationDateTime[5]);
        introObject.getExpiration().setOffset((short) expirationDateTime[6]);
        StringBuffer capabilitiesBuffer = new StringBuffer();
        for(byte ch : capabilities) {
            if(ch >= 32 && ch < 127) {
                capabilitiesBuffer.append((char) ch);
            }
        }
        introObject.setCapabilities(capabilitiesBuffer.toString());
        return new MessageContainer("MobilityIntro", introObject);
    }

}
