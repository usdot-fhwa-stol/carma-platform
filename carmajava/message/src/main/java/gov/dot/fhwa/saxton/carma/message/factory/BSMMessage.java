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
import org.ros.node.ConnectedNode;

import j2735_msgs.BSM;
import cav_msgs.ByteArray;
import gov.dot.fhwa.saxton.carma.message.helper.BSMMessageHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is the actual worker for encoding and decoding BSM message
 * by using J2735 compiler shared library.
 */
public class BSMMessage implements IMessage<BSM> {

    protected ConnectedNode node_;
    protected SaxtonLogger log_;
    protected MessageFactory messageFactory_;

    public BSMMessage(ConnectedNode node, SaxtonLogger log, MessageFactory messageFactory) {
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
		}
	}

	/**
	 * This is the declaration for native method. It will take data from BSM message
     * object and return an byte array with encoded information. Because of the
     * efficiency of JNI method call, it takes fields directly instead of a
     * single BSM object.
     * 
	 * @return encoded BSM message
	 */
	private native byte[] encode_BSM(
			int msgCnt, int[] id, int secMark, int lat, int lon, int elev, int[] accuracy,
			int transmission, int speed, int heading, int angle, int[] acceleration,
			int[] wheel_brakes, int[] vehicle_size);

	/**
	 * This is the declaration for native method. It will take encoded BSM byte array
	 * and a empty BSM object as input. It will decode the message and set all fields in BSM.
	 * Because of the efficiency of JNI method call, it takes different parts of BSM as parameters
	 * instead of a single BSM object.
	 *
	 * @param encoded_array The encoded BSM message
	 * @param plain_msg The empty BSM object
	 * @param bsm_id Decoded id number of the BSM message in order to handle ChannelBuffer type
	 * @param accuracy Decoded positional accuracy set
	 * @param transmission Decoded transmission status set
	 * @param accelset Decoded the acceleration data in 4 ways
	 * @param brakeStatus Decoded the brake status set
	 * @param size Decoded size of the vehicle
	 * @return -1 means decode failed; 0 means decode is successful
	 */
	private native int decode_BSM(byte[] encoded_array, Object plain_msg, byte[] bsm_id, Object accuracy,
			Object transmission, Object accelset, byte[] brakeStatus, Object size);
	
	@Override
    public MessageContainer encode(Message plainMessage) {
        BSMMessageHelper helper_bsm = new BSMMessageHelper(((BSM) plainMessage).getCoreData());
        int[] brakes_status = {
                helper_bsm.getWheel_brakes(), helper_bsm.getTraction(), helper_bsm.getAbs(),
                helper_bsm.getScs(), helper_bsm.getBba(), helper_bsm.getAux()};
        byte[] encode_msg = encode_BSM(
                helper_bsm.getMsgCnt(), helper_bsm.getId(), helper_bsm.getSecMark(),
                helper_bsm.getLat(), helper_bsm.getLon(), helper_bsm.getElev(),
                helper_bsm.getAccuracy(), helper_bsm.getTransmission(), helper_bsm.getSpeed(),
                helper_bsm.getHeading(), helper_bsm.getAngle(), helper_bsm.getAcceleration(),
                brakes_status, helper_bsm.getVehicle_size()
                );
        if(encode_msg == null) {
            log_.error("BSM", "BSMMessage cannot encode bsm message.");
            return new MessageContainer("ByteArray", null);
        }
        ByteArray binary_msg = messageFactory_.newFromType(ByteArray._TYPE);
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
        binary_msg.setContent(buffer);
        binary_msg.setMessageType("BSM");
        binary_msg.getHeader().setFrameId("0");
        binary_msg.getHeader().setStamp(node_.getCurrentTime());
        return new MessageContainer("ByteArray", binary_msg);
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer channelBuffer = binaryMessage.getContent();
        byte[] encoded_bsm = new byte[channelBuffer.capacity()];
        for(int i = 0; i < channelBuffer.capacity(); i++) {
            encoded_bsm[i] = channelBuffer.getByte(i);
        }
        byte[] temp_ID = new byte[4];
        Arrays.fill(temp_ID, (byte) 0);
        byte[] brakeStatus = new byte[6];
        Arrays.fill(brakeStatus, (byte) 0);
        BSM msg_object = messageFactory_.newFromType(BSM._TYPE);
        int result = decode_BSM(
                encoded_bsm, msg_object.getCoreData(),
                temp_ID, msg_object.getCoreData().getAccuracy(),
                msg_object.getCoreData().getTransmission(), msg_object.getCoreData().getAccelSet(),
                brakeStatus, msg_object.getCoreData().getSize()
                );
        if(result == -1) {
            log_.error("BSM", "BSMMessage cannot decode bsm message");
            return new MessageContainer("BSM", null);
        }
        ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, temp_ID);
        msg_object.getCoreData().setId(buffer);
        //Set BrakeAppliedStatus after default shift in asn1c library
        msg_object.getCoreData().getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) (brakeStatus[0] >> 3));
        msg_object.getCoreData().getBrakes().getTraction().setTractionControlStatus(brakeStatus[1]);
        msg_object.getCoreData().getBrakes().getAbs().setAntiLockBrakeStatus(brakeStatus[2]);
        msg_object.getCoreData().getBrakes().getScs().setStabilityControlStatus(brakeStatus[3]);
        msg_object.getCoreData().getBrakes().getBrakeBoost().setBrakeBoostApplied(brakeStatus[4]);
        msg_object.getCoreData().getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus(brakeStatus[5]);
        msg_object.getHeader().setFrameId("MessageConsumer");
        msg_object.getHeader().setStamp(node_.getCurrentTime());
        return new MessageContainer("BSM", msg_object);
    }
}
