package gov.dot.fhwa.saxton.glidepath.asd;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.net.*;

public class AsdInitializer implements IConsumerInitializer {
	
	public AsdInitializer(DatagramSocket sock, long timeout, AsdMessageType msgType) {
		socket_ = sock;
		initTimeout_ = timeout;
        msgType_ = msgType;
	}

	/**
	 * Reads data from the ASD device as necessary to establish a handshake relationship
	 * and prepare for full-frequency operation.
	 * 
	 * Note: socket needs to be open prior to calling this method.
	 * 
	 * Handshake established within timeout budget : true and socket remains open
	 * Timeout expires without success : false and socket is closed
	 */
	@Override
	public Boolean call() throws Exception {
		
		long startTime = System.currentTimeMillis();

		//create a message object to receive any content the device may be sending
		AsdDataPacket data = new AsdDataPacket(socket_, (int)initTimeout_, msgType_);
		
		boolean dataReceived = false;
		boolean timedOut = false;
		do {
			dataReceived = data.read();
			timedOut = (System.currentTimeMillis() - startTime) >= initTimeout_;
		}while (!dataReceived  &&  !timedOut);
		
		//if we received a message in time then
		if (dataReceived) {
			//just log that it happened. No need to worry about the contents now, as the AsdConsumer will 
			//worry about interpreting it.  At this point we already have confidence that we have a 
			//well-formed MAP or SPAT message.
			log_.infof("ASD", "Successful detection of ASD traffic took %d ms.", System.currentTimeMillis() - startTime);
			log_.infof("ASD", "---first bytes: %s", data.toString(20));
			
		//else (no message within timeout period)
		}else {
			//log the fact that we didn't see any data
			log_.debug("ASD", "No recognizable message received from the ASD device during initialization. Continuing.");
		}
		
		return new Boolean(true);
	}
	
	////////////////////
	// member attributes
	////////////////////
	
	private long				initTimeout_;
	private DatagramSocket		socket_;
    private AsdMessageType      msgType_;
	private static ILogger log_ = LoggerManager.getLogger(AsdInitializer.class);
}
