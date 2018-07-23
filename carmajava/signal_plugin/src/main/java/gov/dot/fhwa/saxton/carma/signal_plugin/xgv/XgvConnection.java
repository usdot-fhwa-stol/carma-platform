package gov.dot.fhwa.saxton.carma.signal_plugin.xgv;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamPacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.BitStreamUnpacker;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.AccelerationManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;

import java.io.IOException;
import java.net.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

/**
* Maintains XGV connection state and allows for message transmission and reception via JUDP
 *
 * Transmits and recieves messages in the form of JausMessage objects. These objects may be subclasses if certain
 * command codes are detected in the message header. This should only matter to those looking for specific messages.
*/
public class XgvConnection {

    private DatagramSocket sock;
    private ILogger log;
    private int retryLimit;
    private InetSocketAddress outbound;
    private HashSet<Integer> controlledComponents = new HashSet<>();
    private boolean requestAck = false;

    public XgvConnection(DatagramSocket sock, InetSocketAddress outbound) {
        this.sock = sock;
        this.outbound = outbound;
        this.log = LoggerManager.getLogger(getClass());
        retryLimit = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getJausRetryLimit();
        log.debugf("XGV", "New XgvConnection with retryLimit = %d, sock port = %d, outbound port = %d", retryLimit, sock.getPort(), outbound.getPort());
    }

    public void setRequestAck(boolean val) {
        this.requestAck = val;
    }

    public void setTimeout(int timeout) {
        try {
            sock.setSoTimeout(timeout);
        } catch (SocketException e) {
            log.warn("", "Unable to set XGV socket timeout!");
        }
    }

    public synchronized void sendJausMessage(JausMessage jausMessage) throws IOException {
        if (requestAck) {
            jausMessage.requestAck();
        }

        JausMessage[] judpPacket = {jausMessage};
        sendJausMessages(judpPacket);

        if (requestAck) {
            JausMessage.JausCommandCode responseCode = JausMessage.JausCommandCode.fromInt(jausMessage.getCommandCode());
            if (responseCode != null) {
                if (waitForMessage(responseCode) != null) {
                    log.debugf("", "Received ACK for %d", responseCode.getCode());
                } else {
                    log.debugf("", "Failed to receive ACK for %d", responseCode.getCode());
                }
            } else {
                log.warnf("", "Attempting to get ACK response for unknown JAUS message: %d.",
                          jausMessage.getCommandCode());
            }
        }
    }

    /**
     * Encode an array of up to 5 JausMessage objects into a single JUDP packet for network transmission. Packet size in
     * terms of bytes is not currently enforced but does exist as a constraint on the JUDP protocol.
     * enforced currently, but will be.
     *
     * Synchronized to prevent issues with multiple threads holding a single XgvConnection. Only one thread may attempt
     * to send messages over this XgvConnection at a time. All other XgvConnection operations (except where they call
     * this one) should be thread safe.
     *
     * @param message An array of up to 5 JausMessage objects to be transmitted
     * @throws IOException Thrown by DatagramSocket in case of network errors
     */
    private synchronized void sendJausMessages(JausMessage[] message) throws IOException {
        if (message.length > 5) {
            throw new IllegalStateException("Too many JAUSMessages to generate a valid JUDP packet.");
        }

        BitStreamPacker p = new BitStreamPacker(true);

        // Write JUDP Header
        p.writeByte(0x01); // JUDP version

        // Write the messages
        byte[] serializedMessage;
        for (int i = 0; i < message.length; i++) {
            serializedMessage = message[i].serialize();
            p.writeShort(0); // Compression flags, unused;
            p.setBigEndian();
            p.writeShort(serializedMessage.length);
            p.setLittleEndian();
            p.writeByteArray(serializedMessage);
        }

        byte[] packet = p.getBytes();

        // DGF add logging of packet send contents
        log.debug("XGV", "Sent UDP PacketSize: " + packet.length);
        log.debug("XGV", "Sending UDP PacketData: " + javax.xml.bind.DatatypeConverter.printHexBinary(packet));

        sock.send(new DatagramPacket(packet, packet.length, outbound));
        for (JausMessage m : message) {
            log.info("", "Sent JausMessage with command code: " + m.getCommandCode() + "(0x" + Integer.toHexString(m.getCommandCode()) + ")" + ".");
        }
    }

    /**
     * Waits for a message with JAUS command code commandCode to be recieved. Will block until timeout to receive and
     * attempt up to a fixed number of retries to recieve the message.
     * @param commandCode The command code of the message you want to receive
     * @return A JausMessage object containing the parsed message with the requested commandCode
     * @throws IOException
     */
    public JausMessage waitForMessage(JausMessage.JausCommandCode commandCode) throws IOException {
        int retries = 0;
        JausMessage target = null;
        while (retries < retryLimit && target == null) {
            log.infof("", "Waiting for message %d " + "(0x" + Integer.toHexString(commandCode.getCode()) + ")" + ". Attempt #%d", commandCode.getCode(), retries + 1);
            try {
                JausMessage[] judpMessage = receiveJausMessages();

                // Walk the messages in the JUDP packet
                for (int i = 0; i < judpMessage.length; i++) {
                    if (judpMessage[i].getCommandCode() == commandCode.getCode()) {
                        log.infof("", "Message with command code %d found!", commandCode.getCode());
                        target = new JausMessage(judpMessage[i]);
                    }
                }
            } catch (SocketTimeoutException to) {
                //Pass, we expect this to happen
            } finally {
                retries++;
            }
        }

        // Log failure
        if (target == null) {
            log.warnf("!", "Unable to receive message %d after %d attempts.", commandCode.getCode(), retries + 1);
        }

        return target;
    }

    /**
     * Block while waiting to receive a JUDP packet from the XGV. Does not attempt retries after timeout, up to caller
     * to implement retries or error handling.
     *
     * @return An array containing all the JAUS messages encoded in the most recently received JUDP packet
     * @throws IOException thrown from DatagramSocket for network errors
     */
    public JausMessage[] receiveJausMessages() throws IOException {
        return decodeJudpPacket(getPacket());
    }

    /**
     * Private helper method to assist with receiving of generic UDP packets.
     *
     * Synchronized to prevent issues with multiple threads holding a single XgvConnection. Only one thread may attempt
     * to receive messages over this XgvConnection at a time. All other XgvConnection operations (except where they call
     * this one) should be thread safe.
     *
     * @return a DatagramPacket containing the UDP data received over the network.
     * @throws IOException
     */
    private synchronized DatagramPacket getPacket() throws IOException {
        byte[] payload = new byte[Constants.JAUS_MAX_PACKET_SIZE];
        DatagramPacket packet = new DatagramPacket(payload, Constants.JAUS_MAX_PACKET_SIZE);
        sock.receive(packet);

        // log size and contents of received packet
        log.debug("XGV", "Received UDP Packet Size: " + packet.getLength());
        byte[] packetOnly = Arrays.copyOfRange(packet.getData(), 0, packet.getLength());
        log.debug("XGV", "Received UDP PacketData: " + javax.xml.bind.DatatypeConverter.printHexBinary(packetOnly));

        return packet;
    }

    /**
     * Private helper method to assist with decoding of JUDP packets recieved from the XGV.
     *
     * @param packet The raw UDP packet
     * @return An array of decoded JAUS messages corresponding to all the messages contained in the JUDP packet (up to 5)
     */
    private JausMessage[] decodeJudpPacket(DatagramPacket packet) {
        ArrayList<JausMessage> messages = new ArrayList<JausMessage>();
        BitStreamUnpacker bitstream = new BitStreamUnpacker(packet.getData());
        bitstream.setLittleEndian();

        // Parse JUDP header
        int JUDPVersion = bitstream.readByte();
        bitstream.readShort(); // Unused

        bitstream.setBigEndian();
        int messageLength = bitstream.readShort();
        bitstream.setLittleEndian();

        while (messageLength != 0) {
            messages.add(new JausMessage(bitstream.readByteArray(messageLength)));
            bitstream.readShort(); // Unused

            bitstream.setBigEndian();
            messageLength = bitstream.readShort();
            bitstream.setLittleEndian();
        }

        return messages.toArray(new JausMessage[messages.size()]);
    }

    /**
     * Sends a Component Query Message to the specified JAUS Component ID
     * @param componentJausId An integer representation of the desired component ID
     * @throws IOException Thrown by DatagramSocket
     */
    public void sendComponentStatusQueryMessage(int componentJausId) throws IOException {
        byte[] empty = new byte[0];
        JausMessage queryComponentStatus = new JausMessage(false, componentJausId, empty,
                JausMessage.JausCommandCode.QUERY_COMPONENT_STATUS.getCode());
        sendJausMessage(queryComponentStatus);
        log.info("", "Query Component Status message for component " + componentJausId + " sent to XGV.");
    }

    /**
     * Sends a Query Velocity State Message to the Velocity State Sensor JAUS component
     *
     * @throws IOException Thrown by DatagramSocket
     */
    public void sendQueryVelocityStateMessage() throws IOException {
        int presenceVector = 1;
        BitStreamPacker p = new BitStreamPacker(true);
        p.writeShort(presenceVector);
        int velocityStateSensorId = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getVssJausId();
        JausMessage queryVelocityState = new JausMessage(false, velocityStateSensorId, p.getBytes(),
                JausMessage.JausCommandCode.QUERY_VELOCITY_STATE.getCode());
        sendJausMessage(queryVelocityState);
        log.info("", "Query Velocity State message sent to XGV.");
    }

    /**
     * Sends a Query Discrete Device State message to the XGV, requesting a Report Discrete Device State response.
     * @throws IOException thrown by DatagramSocket in the event of a network error
     */
    public void sendDiscreteDeviceStateQuery() throws IOException {
        int presenceVector = 0b1111;
        byte[] body = { (byte) presenceVector};
        int primitiveDriverId = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getPdJausId();
        JausMessage queryDiscreteDevices = new JausMessage(false, primitiveDriverId, body,
                JausMessage.JausCommandCode.QUERY_DISCRETE_DEVICES.getCode());
        sendJausMessage(queryDiscreteDevices);
        log.info("", "Query Discrete Devices message sent to XGV.");
    }

    /**
     * Sends a Query Component Control message to the XGV requesting a Report Component Control response.
     * @param targetJausId The JAUS ID of the component we wish to send the message to.
     */
    public void sendQueryComponentControl(int targetJausId) throws IOException {
        byte[] empty = new byte[0];
        JausMessage queryComponentControl = new JausMessage(false, targetJausId, empty,
                JausMessage.JausCommandCode.QUERY_COMPONENT_CONTROL.getCode());
        sendJausMessage(queryComponentControl);
    }

    /**
     * Sends a Request Component Control message to the XGV. Control of the component will be granted if we have a higher
     * authority than the current controlling component or there is no current controlling component.
     * @param targetJausId The JAUS ID of the component we wish to send the message to.
     * @param authority The authority level to be used in the request.
     */
    public void sendRequestComponentControl(int targetJausId, int authority) throws IOException {
        byte[] payload = new byte[1];
        payload[0] = (byte) authority;
        JausMessage requestComponentControl = new JausMessage(false,targetJausId,payload,
                                                    JausMessage.JausCommandCode.REQUEST_COMPONENT_CONTROL.getCode());

        controlledComponents.add(targetJausId);
        sendJausMessage(requestComponentControl);
    }


    /**
     * Sends an empty payload message to the specified JAUS Component ID
     * @param componentJausId An integer representation of the desired component ID
     * @param code RESUME, STANDBY, etc.                       @
     * @throws IOException Thrown by DatagramSocket
     */
    public void sendEmptyPayloadMessage(int componentJausId, JausMessage.JausCommandCode code) throws IOException {
        byte[] empty = new byte[0];
        JausMessage queryComponentStatus = new JausMessage(false, componentJausId, empty,
                code.getCode());
        sendJausMessage(queryComponentStatus);
        log.info("", "Sent empty message with code " + code + " message to  " + componentJausId + " to XGV.");
    }

    /**
     * Sends a Set Motion Profile message to the Motion Profile Controller on the XGV. Commands the vehicle to attain a
     * the specified speed and trajectory. Must have control of the Motion Profile Driver first via {@link #sendRequestComponentControl}.
     * @param velocity
     * @throws IOException
     */
    public void sendSetMotionProfile(double velocity) throws IOException {
        GlidepathAppConfig appConfig = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig());

        if (!controlledComponents.contains(appConfig.getMpdJausId())) {
            log.warn("", "Attempting to send Motion Profile to XGV without first taking control.");
        }

        BitStreamPacker p = new BitStreamPacker(true);

        int numFrames = Integer.valueOf(appConfig.getProperty("xgv.numframes"));
        int frameDelay = Integer.valueOf(appConfig.getProperty("xgv.framedelay"));
        
        p.writeByte(1); // Unused, constant set to 1
        p.writeByte(numFrames); // Number of motions

        for (int i = 0;  i < numFrames;  ++i) {
        	appendMotionProfileFrame(velocity, frameDelay, p);
        }

        byte[] body = p.getBytes();

        JausMessage setMotionProfileMessage = new JausMessage(true, appConfig.getMpdJausId(), body,
                                                              JausMessage.JausCommandCode.SET_MOTION_PROFILE.getCode());

        sendJausMessage(setMotionProfileMessage);
    }

    /**
     * Append the data necessary for an additional motion profile frame to the BitStreamPacker. Can be used to generate
     * longer motion profile messages.
     *
     * @param velocity the target velocity to be attained during the motion profile frame
     * @param durationMs the duration of the motion profile frame
     * @param p the bitstream packer containing the initial setup of the SendMotionProfile message and any previous
     *          motion profile frames.
     */
    private void appendMotionProfileFrame(double velocity, int durationMs, BitStreamPacker p) {
        int scaledVel = XgvIntegerConverter.realToUnsignedScaled(velocity, 65.535, -65.535, 16);
        p.writeShort(scaledVel);
        double maxAccel = AccelerationManager.getManager().getAccelLimit();
        int scaledMaxAccel = XgvIntegerConverter.realToUnsignedScaled(maxAccel, 100, 0, 16);
        p.writeShort(scaledMaxAccel);

        // Used for steering control, not needed
        p.writeShort(XgvIntegerConverter.realToUnsignedScaled(0, Math.PI/2, -Math.PI/2, 16 ));
        p.writeShort(XgvIntegerConverter.realToUnsignedScaled(5, 10, 0, 16));

        p.writeShort(durationMs);
    }

    /**
     * Send a Release Component Control Message to an Xgv JAUS Component. If the component is not controlled nothing
     * happens.
     * @param targetJausId The ID of the component to release control of.
     * @throws IOException
     */
    public void sendReleaseComponentControl(int targetJausId) throws IOException {
        byte[] empty = new byte[0];
        JausMessage releaseComponentControl = new JausMessage(false, targetJausId, empty,
                        JausMessage.JausCommandCode.RELEASE_COMPONENT_CONTROL.getCode());
        sendJausMessage(releaseComponentControl);
        controlledComponents.remove(targetJausId);
    }

    /**
     * Close the sockets associated with this XgvConnection. Also relinquishes control of any components we currently
     * have control of.
     */
    public void close() {
        // Clone the set of controlled components and release them
        HashSet<Integer> temp = new HashSet<>(controlledComponents);
        for (int jausID : temp) {
            try {
                sendReleaseComponentControl(jausID);
            } catch (IOException e) {
                log.warn("", "Unable to properly release control of component " + jausID + "before closing XgvConnection.");
            }
        }

        sock.disconnect();
        sock.close();
    }
}
