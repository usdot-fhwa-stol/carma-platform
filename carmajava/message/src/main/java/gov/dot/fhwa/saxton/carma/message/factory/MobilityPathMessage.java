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
import cav_msgs.MobilityPath;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityPathHelper;
import gov.dot.fhwa.saxton.carma.message.helper.MobilityTrajectoryHelper;
import gov.dot.fhwa.saxton.carma.message.helper.StringConverterHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding Mobility Path message
 * by using J2735 compiler shared library.
 */
public class MobilityPathMessage implements IMessage<MobilityPathMessage> {

        private MessageFactory factory;
        private SaxtonLogger log;

        public MobilityPathMessage(SaxtonLogger log, MessageFactory factory) {
                this.factory = factory;
                this.log = log;
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
         * This is the declaration for native method. It will take data from MobilityPath ROS message
         * and return a byte array including the encoded message. Because of the efficiency of JNI method
         * call, it takes fields directly instead of a single MobilityPath object.
         * 
         * @return encoded MobilityPath Message
         */
        private native byte[] encodeMobilityPath(byte[] senderId, byte[] targetId, byte[] senderBSMId, byte[] planId,
                        byte[] timestamp, int currentX, int currentY, int currentZ, byte[] locationTimestamp,
                        int[][] offsets);

        /**
         * This is the declaration for native method. It will take encoded MobilityPath byte array
         * and a empty MobilityPath object as input. It will decode the message and set all fields in MobilityPath.
         * Because of the efficiency of JNI method call, it takes different parts of MobilityPath as parameters
         * instead of a single MobilityPath object.
         * @return -1 means decode failed; 0 means decode is successful
         */
        public native int decodeMobilityPath(byte[] encodedArray, Object mobilityPath, byte[] senderId, byte[] targetId,
                        byte[] bsmId, byte[] planId, byte[] timestamp, Object currentLocation, byte[] locationTimestamp,
                        int[][] offsets);

        public byte[] callJniEncode(MobilityPath message) {
                MobilityPathHelper helper = new MobilityPathHelper(message);
                return encodeMobilityPath(helper.getHeaderHelper().getSenderId(),
                                helper.getHeaderHelper().getTargetId(), helper.getHeaderHelper().getBSMId(),
                                helper.getHeaderHelper().getPlanId(), helper.getHeaderHelper().getTimestamp(),
                                helper.getTrajectoryHelper().getStartLocationHelper().getEcefX(),
                                helper.getTrajectoryHelper().getStartLocationHelper().getEcefY(),
                                helper.getTrajectoryHelper().getStartLocationHelper().getEcefZ(),
                                helper.getTrajectoryHelper().getStartLocationHelper().getTimestamp(),
                                helper.getTrajectoryHelper().getOffsets());
        }

        @Override
        public MessageContainer encode(Message plainMessage) {
                byte[] encodedMsg = callJniEncode((MobilityPath) plainMessage);
                if (encodedMsg == null) {
                        log.warn("MobilityRequest", "MobilityPathMessage cannot encode the message");
                        return new MessageContainer("ByteArray", null);
                }
                ByteArray binaryMsg = factory.newFromType(ByteArray._TYPE);
                ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encodedMsg);
                binaryMsg.setContent(buffer);
                binaryMsg.setMessageType("MobilityPath");
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
                byte[] trajectoryStartTime = new byte[19];
                int[][] offsets = new int[3][60];
                // fill with character 'zero'
                Arrays.fill(trajectoryStartTime, (byte) 48);
                // fill with unavailable data 
                Arrays.fill(offsets[0], 501);
                Arrays.fill(offsets[1], 501);
                Arrays.fill(offsets[2], 501);
                MobilityPath path = factory.newFromType(MobilityPath._TYPE);
                int result = decodeMobilityPath(encodedMsg, path, senderId, targetId, bsmId, planId, timestamp,
                                path.getTrajectory().getLocation(), trajectoryStartTime, offsets);
                if (result == -1) {
                        log.warn("MobilityPathMessage cannot decode message.");
                        return new MessageContainer("MobilityPath", null);
                }
                path.getHeader().setSenderId(StringConverterHelper.readDynamicLengthString(senderId));
                path.getHeader().setRecipientId(StringConverterHelper.readDynamicLengthString(targetId));
                path.getHeader().setSenderBsmId(new String(bsmId));
                path.getHeader().setPlanId(new String(planId));
                path.getHeader().setTimestamp(Long.parseLong(new String(timestamp)));
                path.getTrajectory().getLocation().setTimestamp(Long.parseLong(new String(trajectoryStartTime)));
                MobilityTrajectoryHelper helper = new MobilityTrajectoryHelper(factory);
                path.getTrajectory().setOffsets(helper.intArrayOffsetsToOffsetList(offsets));
                return new MessageContainer("MobilityPath", path);
        }

}