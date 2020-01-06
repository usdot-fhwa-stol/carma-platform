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
import cav_msgs.MobilityOperation;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityHeaderHelper;
import gov.dot.fhwa.saxton.carma.message.helper.StringConverterHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility
 * Operation message by using J2735 compiler shared library.
 */
public class MobilityOperationMessage implements IMessage<MobilityOperation> {

    protected static final int STRATEGY_MAX_LENGTH = 50;
    protected static final int STRATEGY_PARAMS_MAX_LENGTH = 100;
    
    private MessageFactory factory;
    private SaxtonLogger   log;
    
    public MobilityOperationMessage(MessageFactory factory, SaxtonLogger log) {
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
     * This is the declaration for native method. It will take data from MobilityOperation ROS message
     * and return a byte array including the encoded message. Because of the efficiency of JNI method
     * call, it takes fields directly instead of a single MobilityOperation object.
     * 
     * @return encoded MobilityOperation Message
     */
    private native byte[] encodeMobilityOperation(byte[] senderId, byte[] targetId, byte[] senderBSMId,
            byte[] planId, byte[] timestamp, byte[] strategy, byte[] params);
    
    /**
     * This is the declaration for native method. It will take encoded MobilityOperation byte array as input.
     * It will decode the message and set all string fields in other byte array inputs.
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMobilityOperation(byte[] encodedArray, byte[] senderId, byte[] targetId, byte[] bsmId,
            byte[] planId, byte[] timestamp, byte[] strategy, byte[] params);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        byte[] encodedMsg = callJniEncode((MobilityOperation) plainMessage);
        if (encodedMsg == null) {
            log.warn("MobilityOperation", "MobilityOperationMessage cannot encode the message");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binaryMsg = factory.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encodedMsg);
        binaryMsg.setContent(buffer);
        binaryMsg.setMessageType("MobilityOperation");
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
        byte[] strategy = new byte[50];
        byte[] strategyParams = new byte[100];
        // fill with character 'zero'
        Arrays.fill(timestamp, (byte) 48);
        MobilityOperation operation = factory.newFromType(MobilityOperation._TYPE);
        int result = decodeMobilityOperation(encodedMsg, senderId, targetId, bsmId, planId, timestamp, strategy, strategyParams);
        if (result == -1) {
                log.warn("MobilityOperationMessage cannot decode message.");
                return new MessageContainer("MobilityOperation", null);
        }
        operation.getHeader().setSenderId(StringConverterHelper.readDynamicLengthString(senderId));
        operation.getHeader().setRecipientId(StringConverterHelper.readDynamicLengthString(targetId));
        operation.getHeader().setSenderBsmId(new String(bsmId));
        operation.getHeader().setPlanId(new String(planId));
        operation.getHeader().setTimestamp(Long.parseLong(new String(timestamp)));
        operation.setStrategy(StringConverterHelper.readDynamicLengthString(strategy));
        operation.setStrategyParams(StringConverterHelper.readDynamicLengthString(strategyParams));
        return new MessageContainer("MobilityOperation", operation);
    }

    public byte[] callJniEncode(MobilityOperation msg) {
        MobilityHeaderHelper header = new MobilityHeaderHelper(msg.getHeader());
        return encodeMobilityOperation(header.getSenderId(), header.getTargetId(),
                            header.getBSMId(), header.getPlanId(), header.getTimestamp(),
                            StringConverterHelper.setDynamicLengthString(msg.getStrategy(), STRATEGY_MAX_LENGTH),
                            StringConverterHelper.setDynamicLengthString(msg.getStrategyParams(), STRATEGY_PARAMS_MAX_LENGTH));
    }
    
}
