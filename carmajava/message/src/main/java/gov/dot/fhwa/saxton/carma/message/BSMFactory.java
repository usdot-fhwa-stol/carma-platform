package gov.dot.fhwa.saxton.carma.message;

import java.nio.ByteOrder;
import java.util.Arrays;

import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

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
	 * This the declaration for native method. It will take data from BSM message
	 * object and return an byte array with encoded information. Because of the
	 * efficiency of JNI method call, it takes different parts of BSM instead of a
	 * single BSM object.
	 * 
	 * @param bsm_core the BSMCoreData object of the BSM object
	 * @param bsm_id  id number of the BSM message in order to handle ChannelBuffer type
	 * @param accuracy the positional accuracy set
	 * @param transmission the transmission status set
	 * @param accelset the acceleration data in 4 ways
	 * @param brakeStatus the brake status set
	 * @param size size of the vehicle
	 * @return encoded BSM message
	 */
	private static native byte[] encode_BSM(Object bsm_core, byte[] bsm_id, Object accuracy, Object transmission,
			Object accelset, byte[] brakeStatus, Object size);

	private static native void decode_BSM(byte[] encoded_array, Object plain_msg);
	/**
	 * This method is used by MessageConsumer. It takes whole BSM message object
	 * and an empty ByteArray message. After calling this method,
	 * the content of the empty byte array will be the encoded information of BSM.
	 * 
	 * @param plain_msg whole BSM object
	 * @param skeleton the empty ByteArray object
	 */
	public static void encode(BSM plain_msg, ByteArray skeleton, Log log) {
		byte[] brakeStatus = new byte[] {
				plain_msg.getCoreData().getBrakes().getWheelBrakes().getBrakeAppliedStatus(),
				plain_msg.getCoreData().getBrakes().getTraction().getTractionControlStatus(),
				plain_msg.getCoreData().getBrakes().getAbs().getAntiLockBrakeStatus(),
				plain_msg.getCoreData().getBrakes().getScs().getStabilityControlStatus(),
				plain_msg.getCoreData().getBrakes().getBrakeBoost().getBrakeBoostApplied(),
				plain_msg.getCoreData().getBrakes().getAuxBrakes().getAuxiliaryBrakeStatus()
				};
		log.info("BSMFactory: Start to encode bsm message");
		byte[] encode_msg = encode_BSM(
				plain_msg.getCoreData(),
				new byte[] {
						plain_msg.getCoreData().getId().getByte(0), plain_msg.getCoreData().getId().getByte(1),
						plain_msg.getCoreData().getId().getByte(2), plain_msg.getCoreData().getId().getByte(3)
						},
				plain_msg.getCoreData().getAccuracy(), plain_msg.getCoreData().getTransmission(),
				plain_msg.getCoreData().getAccelSet(), brakeStatus, plain_msg.getCoreData().getSize());
		skeleton.setMessageType("BSM");
		ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
		skeleton.setContent(buffer);
	}
	
	public static void decode(ByteArray encoded_msg, BSM skeleton, Log log) {
		ChannelBuffer channelBuffer = encoded_msg.getContent();
		byte[] encoded_bsm = channelBuffer.toByteBuffer().array();
		log.info("Here!!!!!!!!!!!!!!" + Arrays.toString(encoded_bsm));
		decode_BSM(encoded_bsm, skeleton.getCoreData());
	}
}
