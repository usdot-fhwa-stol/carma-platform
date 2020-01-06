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
import cav_msgs.MobilityRequest;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityRequestHelper;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityTrajectoryHelper;
import gov.dot.fhwa.saxton.carma.message.helper.StringConverterHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility Request message
 * by using J2735 compiler shared library.
 */
public class MobilityRequestMessage implements IMessage<MobilityRequestMessage>{
    
    protected SaxtonLogger log_;
    protected MessageFactory messageFactory_;
    
    public MobilityRequestMessage(SaxtonLogger log, MessageFactory messageFactory) {
        this.log_ = log;
        this.messageFactory_ = messageFactory;
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
     * This is the declaration for native method. It will take data from MobilityRequest ROS message
     * and return a byte array including the encoded message. Because of the efficiency of JNI method
     * call, it takes fields directly instead of a single MobilityRequest object.
     * 
     * @return encoded MobilityRequest Message
     */
    private native byte[] encodeMobilityRequest(
            byte[] senderId, byte[] targetId, byte[] senderBSMId, byte[] planId, byte[] timestamp, byte[] strategy,
            int planType, int urgency, int currentX, int currentY, int currentZ, byte[] currentT, byte[] strategyParams,
            int startX, int startY, int startZ, byte[] startT, int[][] offsets, byte[] expiration);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityIntro byte array
     * and a empty MobilityIntro object as input. It will decode the message and set all fields in MobilityRequest.
     * Because of the efficiency of JNI method call, it takes different parts of MobilityRequest as parameters
     * instead of a single MobilityRequest object.
     * @return -1 means decode failed; 0 means decode is successful
     */
    private native int decodeMobilityRequest(
            byte[] encodedArray, Object mobilityReq, byte[] senderId, byte[] targetId,
            byte[] bsmId, byte[] planId, byte[] timestamp, byte[] strategy, Object planType,
            Object currentLocation, byte[] locationTime, byte[] strategyParams, Object trajectoryStartLocation,
            byte[] trajectoryStartTime, int[][] offsets, byte[] expiration);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        byte[] encodedMsg = this.callJniEncode((MobilityRequest) plainMessage);
        if(encodedMsg == null) {
            log_.warn("MobilityRequest", "MobilityRequestMessage cannot encode the message");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binaryMsg = messageFactory_.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encodedMsg);
        binaryMsg.setContent(buffer);
        binaryMsg.setMessageType("MobilityRequest");
        binaryMsg.getHeader().setFrameId("0");
        binaryMsg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
        return new MessageContainer("ByteArray", binaryMsg);
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encodedMsg = new byte[buffer.capacity()];
        for(int i = 0; i < buffer.capacity(); i++) {
            encodedMsg[i] = buffer.getByte(i);
        }
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] strategy = new byte[50];
        byte[] locationTime = new byte[19];
        byte[] strategyParams = new byte[100];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] expiration = new byte[19];
        // fill with character 'zero'
        Arrays.fill(timestamp, (byte) 48);
        Arrays.fill(trajectoryStartTime, (byte) 48);
        Arrays.fill(expiration, (byte) 48);
        // fill with unavailable data 
        Arrays.fill(offsets[0], 501);
        Arrays.fill(offsets[1], 501);
        Arrays.fill(offsets[2], 501);
        MobilityRequest request = messageFactory_.newFromType(MobilityRequest._TYPE);
        int result = callJniDecode(
                encodedMsg, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                request.getPlanType(), request.getLocation(), locationTime, strategyParams,
                request.getTrajectory().getLocation(), trajectoryStartTime, offsets, expiration);
        if(result == -1) {
            log_.warn("MobilityRequest", "MobilityRequestMessage cannot decode message.");
            return new MessageContainer("MobilityRequest", null);
        }
        request.getHeader().setSenderId(StringConverterHelper.readDynamicLengthString(senderId));
        request.getHeader().setRecipientId(StringConverterHelper.readDynamicLengthString(targetId));
        request.getHeader().setSenderBsmId(new String(bsmId));
        request.getHeader().setPlanId(new String(planId));
        request.getHeader().setTimestamp(Long.parseLong(new String(timestamp)));
        request.setStrategy(StringConverterHelper.readDynamicLengthString(strategy));
        request.getLocation().setTimestamp(Long.parseLong(new String(locationTime)));
        request.setStrategyParams(StringConverterHelper.readDynamicLengthString(strategyParams));
        request.getTrajectory().getLocation().setTimestamp(Long.parseLong(new String(trajectoryStartTime)));
        MobilityTrajectoryHelper helper = new MobilityTrajectoryHelper(messageFactory_);
        request.getTrajectory().setOffsets(helper.intArrayOffsetsToOffsetList(offsets));
        request.setExpiration(Long.parseLong(new String(expiration)));
        return new MessageContainer("MobilityRequest", request);
    }
    
    public byte[] callJniEncode(MobilityRequest request) {
        MobilityRequestHelper helper = new MobilityRequestHelper(request);
        byte[] encodedMsg = encodeMobilityRequest(
                helper.getHeaderHelper().getSenderId(), helper.getHeaderHelper().getTargetId(),
                helper.getHeaderHelper().getBSMId(), helper.getHeaderHelper().getPlanId(),
                helper.getHeaderHelper().getTimestamp(), helper.getStrategy(), helper.getPlanType(),
                helper.getUrgency(), helper.getLocationHelper().getEcefX(), helper.getLocationHelper().getEcefY(),
                helper.getLocationHelper().getEcefZ(), helper.getLocationHelper().getTimestamp(),
                helper.getStrategyParams(), helper.getTrajectoryHelper().getStartLocationHelper().getEcefX(),
                helper.getTrajectoryHelper().getStartLocationHelper().getEcefY(),
                helper.getTrajectoryHelper().getStartLocationHelper().getEcefZ(),
                helper.getTrajectoryHelper().getStartLocationHelper().getTimestamp(),
                helper.getTrajectoryHelper().getOffsets(), helper.getExpiration());
        return encodedMsg;
    }
    
    public int callJniDecode(byte[] encodedArray, Object mobilityReq, byte[] senderId, byte[] targetId,
            byte[] bsmId, byte[] planId, byte[] timestamp, byte[] strategy, Object planType,
            Object currentLocation, byte[] locationTime, byte[] strategyParams, Object trajectoryStartLocation,
            byte[] trajectoryStartTime, int[][] offsets, byte[] expiration) {
        int res = decodeMobilityRequest(
                encodedArray, mobilityReq, senderId, targetId, bsmId, planId, timestamp,
                strategy, planType, currentLocation, locationTime, strategyParams,
                trajectoryStartLocation, trajectoryStartTime, offsets, expiration);
        return res;
    }
}
