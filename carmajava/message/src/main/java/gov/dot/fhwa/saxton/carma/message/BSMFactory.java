package gov.dot.fhwa.saxton.carma.message;

import java.nio.ByteOrder;

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
			System.out.println("Exception trapped while trying to load the asn1c libtaty" + e.toString());
			e.printStackTrace();
		}
	}

	// Declare native methods
	private static native byte[] encode_BSM(Object bsm_core, Object accuracy, Object transmission, Object accelset,
			byte[] brakeStatus, Object size);

	public static void encode(BSM plain_msg, ByteArray skelecton) {
		byte[] brakeStatus = new byte[] { plain_msg.getCoreData().getBrakes().getWheelBrakes().getBrakeAppliedStatus(),
				plain_msg.getCoreData().getBrakes().getTraction().getTractionControlStatus(),
				plain_msg.getCoreData().getBrakes().getAbs().getAntiLockBrakeStatus(),
				plain_msg.getCoreData().getBrakes().getScs().getStabilityControlStatus(),
				plain_msg.getCoreData().getBrakes().getBrakeBoost().getBrakeBoostApplied(),
				plain_msg.getCoreData().getBrakes().getAuxBrakes().getAuxiliaryBrakeStatus() };
		byte[] encode_msg = encode_BSM(plain_msg.getCoreData(), plain_msg.getCoreData().getAccuracy(),
				plain_msg.getCoreData().getTransmission(), plain_msg.getCoreData().getAccelSet(), brakeStatus,
				plain_msg.getCoreData().getSize());
		skelecton.setMessageType("BSM");
		ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, encode_msg);
		skelecton.setContent(buffer);
	}
}
