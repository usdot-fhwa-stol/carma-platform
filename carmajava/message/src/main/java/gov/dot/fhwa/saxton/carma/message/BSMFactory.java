package gov.dot.fhwa.saxton.carma.message;

import java.nio.ByteOrder;
import java.util.Arrays;

import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.node.ConnectedNode;

import cav_msgs.BSM;
import cav_msgs.ByteArray;

public class BSMFactory {

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
	 * This is the declaration for native method. It will take data from BSM message
	 * object and return an byte array with encoded information. Because of the
	 * efficiency of JNI method call, it takes different parts of BSM instead of a
	 * single BSM object.
	 * 
	 * @param bsm_core The BSMCoreData object of the BSM object
	 * @param bsm_id  The id number of the BSM message in order to handle ChannelBuffer type
	 * @param accuracy The positional accuracy set
	 * @param transmission The transmission status set
	 * @param accelset The acceleration data in 4 ways
	 * @param brakeStatus The brake status set
	 * @param size Size of the vehicle
	 * @return encoded BSM message
	 */
	private static native byte[] encode_BSM(Object bsm_core, byte[] bsm_id, Object accuracy, Object transmission,
			Object accelset, byte[] brakeStatus, Object size);

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
	
	private static native int decode_BSM(byte[] encoded_array, Object plain_msg, byte[] bsm_id, Object accuracy,
			Object transmission, Object accelset, byte[] brakeStatus, Object size);
	/**
	 * This method is used by MessageConsumer. It takes whole BSM message object
	 * and an empty ByteArray message. After calling this method,
	 * the content of the empty byte array will be the encoded information of BSM.
	 * 
	 * @param plain_msg Entire BSM object
	 * @param skeleton The empty ByteArray object
	 * @param log Logging any necessary messages
	 * @param node ConnectedNode helps to set message header
	 */
	public static void encode(BSM plain_msg, ByteArray skeleton, Log log, ConnectedNode node) {
		byte[] brakeStatus = new byte[] {
				plain_msg.getCoreData().getBrakes().getWheelBrakes().getBrakeAppliedStatus(),
				plain_msg.getCoreData().getBrakes().getTraction().getTractionControlStatus(),
				plain_msg.getCoreData().getBrakes().getAbs().getAntiLockBrakeStatus(),
				plain_msg.getCoreData().getBrakes().getScs().getStabilityControlStatus(),
				plain_msg.getCoreData().getBrakes().getBrakeBoost().getBrakeBoostApplied(),
				plain_msg.getCoreData().getBrakes().getAuxBrakes().getAuxiliaryBrakeStatus()
				};
		log.info("BSMFactory: Start to encode bsm message");
		byte[] temp_ID = new byte[4];
		for(int i = 0; i < plain_msg.getCoreData().getId().capacity(); i++) {
			temp_ID[i] = plain_msg.getCoreData().getId().getByte(i);
		}
		byte[] encode_msg = encode_BSM(
				plain_msg.getCoreData(), temp_ID,
				plain_msg.getCoreData().getAccuracy(), plain_msg.getCoreData().getTransmission(),
				plain_msg.getCoreData().getAccelSet(), brakeStatus, plain_msg.getCoreData().getSize());
		skeleton.setMessageType("BSM");
		ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
		skeleton.setContent(buffer);
		skeleton.setMessageType("BSM");
		skeleton.getHeader().setFrameId("MessageConsumer");
		skeleton.getHeader().getStamp().secs = node.getCurrentTime().secs;
		skeleton.getHeader().getStamp().nsecs = node.getCurrentTime().nsecs;
	}
	
	/**
	 * This method is used by MessageConsumer. It takes empty BSM message object
	 * and an ByteArray message as encoded BSM. After calling this method,
	 * the content of the empty BSM will be the decoded results.
	 * 
	 * @param encoded_msg The encoded BSM message as a binary array
	 * @param skeleton The empty BSM object
	 * @param log Logging any necessary messages
	 * @param node ConnectedNode helps to set message header
	 * @return
	 */
	public static int decode(ByteArray encoded_msg, BSM skeleton, Log log, ConnectedNode node) {
		ChannelBuffer channelBuffer = encoded_msg.getContent();
		byte[] encoded_bsm = new byte[channelBuffer.capacity()];
		for(int i = 0; i < channelBuffer.capacity(); i++) {
			encoded_bsm[i] = channelBuffer.getByte(i);
		}
		byte[] temp_ID = new byte[4];
		Arrays.fill(temp_ID, (byte) 0);
		byte[] brakeStatus = new byte[6];
		Arrays.fill(brakeStatus, (byte) 0);
		int result = decode_BSM(
				encoded_bsm, skeleton.getCoreData(),
				temp_ID, skeleton.getCoreData().getAccuracy(),
				skeleton.getCoreData().getTransmission(), skeleton.getCoreData().getAccelSet(),
				brakeStatus, skeleton.getCoreData().getSize()
				);
		ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, temp_ID);
		skeleton.getCoreData().setId(buffer);
		//Set BrakeAppliedStatus after default shift in asn1c library
		skeleton.getCoreData().getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) (brakeStatus[0] >>> 3));
		skeleton.getCoreData().getBrakes().getTraction().setTractionControlStatus(brakeStatus[1]);
		skeleton.getCoreData().getBrakes().getAbs().setAntiLockBrakeStatus(brakeStatus[2]);
		skeleton.getCoreData().getBrakes().getScs().setStabilityControlStatus(brakeStatus[3]);
		skeleton.getCoreData().getBrakes().getBrakeBoost().setBrakeBoostApplied(brakeStatus[4]);
		skeleton.getCoreData().getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus(brakeStatus[5]);
		skeleton.getHeader().setFrameId("MessageConsumer");
		skeleton.getHeader().getStamp().secs = node.getCurrentTime().secs;
		skeleton.getHeader().getStamp().nsecs = node.getCurrentTime().nsecs;
		return result;
	}
}
