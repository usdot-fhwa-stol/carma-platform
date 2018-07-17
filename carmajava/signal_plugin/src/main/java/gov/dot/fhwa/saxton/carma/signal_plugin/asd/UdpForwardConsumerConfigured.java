/**
 * UDP Forward Cnosumer
 *
 * Abstract previous AsdConsumer to provide framework for both MAP and SPAT messages
 *
 * User: ferenced
 * Date: 1/16/15
 * Time: 2:48 PM
 *
 */
package gov.dot.fhwa.saxton.carma.signal_plugin.asd;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.glidepath.appcommon.IAsdListDataElement;
import gov.dot.fhwa.saxton.glidepath.appcommon.IntDataElement;
import gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers.SpatUtilConsumer;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;

/**
 * clone of UdpForwardConsumer that just hard codes configuration for SpatUtil app
 *
 */
public abstract class UdpForwardConsumerConfigured implements IConsumerTask {

    public UdpForwardConsumerConfigured() {

        //get timeout budgets from the app config
        boolean perfCheck = true;
        if (perfCheck) {
            initTimeout_ = 10000;
            operTimeout_ = 40;
        }else {
            initTimeout_ = 8*3600000; //N hours, to allow for a detailed debugging session
            operTimeout_ = 8*3600000;
        }
        assert(initTimeout_ > 0);
        assert(operTimeout_ > 0);

        msgNeverParsed_ = true;
        prevVersion_ = 0;

    }

    /**
     * Subclass specific processing of message type
     *
     * @param list - an existing list of messages to be appended to
     * @param message     A complete message parsed from a raw ASD packet
     * @return the updated list of messages
     */
    public abstract List<IAsdMessage> processMessage(List<IAsdMessage> list, IAsdMessage message);


    @Override
    /**
     * Establishes a basic connection and handshake relationship with the ASD device
     *
     * Handshake established within timeout budget : true
     * Timeout budget expires without success : false
     */
    public boolean initialize() {

        try {
            //establish the UDP socket
            socket_ = new DatagramSocket(port_);

            //set up the initializer
            initializer_ = new AsdInitializer(socket_, initTimeout_, msgType_);
        } catch (SocketException e) {
            initializer_ = null;
            terminate();
            log_.errorf("ASD", "Unable to create a UDP socket for the ASD on port %d", port_);
        }

        return initializer_ != null;
    }

    @Override
    /**
     * Always : shut down the device connection
     */
    public void terminate() {

        if (socket_ != null) {
            socket_.close();
        }
    }

    @Override
    public IConsumerInitializer getInitializer() {

        return initializer_;
    }

    @Override
    /**
     * Collects the most recently available MAP & SPAT messages from the ASD device.
     *
     * MAP or SPAT available : returns available message(s)
     * No message retrieved within timeout budget : empty data element holder
     */
    public DataElementHolder call() throws Exception {
        List<IAsdMessage> msgList = new ArrayList<>();

        //get current system time - start of local processing logic
        long startTime = System.currentTimeMillis();

        //get all available messages from the device
        List<AsdDataPacket> dataList = getPackets(socket_, operTimeout_, msgType_);

        //loop through all data packets received
        for (int m = 0;  m < dataList.size();  ++m){

            //if a valid message was received then
            IAsdMessage msg = null;
            AsdDataPacket data = dataList.get(m);
            if (data != null) {

                //create a new message object and parse it
                msg = AsdMessageFactory.newInstance(msgType_);

                try {
                    byte[] buf = data.getBuffer();
                    if (msg.parse(buf)) {
                        int intersectionId = msg.getIntersectionId();
                        // add the pertinent message to the list of messages to be returned
                        msgList = processMessage(msgList, msg);
                        log_.debugf("ASD", "call: parsed the message of type %s for intersections %d",
                                msgType_.toString(), intersectionId);
                    }
                } catch (Exception e) {
                    log_.warnf("ASD", "Exception trapped from msg.parse(): %s", e.toString());
                }
            }
        }

        DataElementHolder rtn = new DataElementHolder();
        Duration duration = new Duration(new DateTime(startTime), new DateTime());
        if (this instanceof SpatUtilConsumer)  {
            rtn.put(DataElementKey.SPAT_LIST, new IAsdListDataElement(msgList));
            rtn.put(DataElementKey.CYCLE_SPAT, new IntDataElement((int) duration.getMillis()));
        }
        log_.infof("ASD", "call: completed for msgType = %s in %d ms with %d messages processed.",
                msgType_.toString(), (System.currentTimeMillis() - startTime), msgList.size());

        return rtn;
    }

    public void setPort(int port)   {
        this.port_ = port;
    }

    public void setMsgType(AsdMessageType msgType)   {
        this.msgType_ = msgType;
    }

    ////////////////////
    // member attributes
    ////////////////////

    /**
     * Gets all data packets of the specified type that were already on the socket and those that appear
     * on the socket within timeout span of time.
     * No data packets queued up and none received in timeout period : empty list
     *
     * @param sock - the socket to be listened to
     * @param timeout - the timeout in ms
     * @param type - the type of ASD message we are listening for
     * @return list of message packets received
     */
    protected List<AsdDataPacket> getPackets(DatagramSocket sock, int timeout, AsdMessageType type) throws Exception {
        List<AsdDataPacket> packets = new ArrayList<>();

        //start the timer
        long startTime = System.currentTimeMillis();

        //Note: there may be multiple packets queued up on the UDP port. If this is the case then it will take only
        // a couple ms to pull off each one. Once they are all cleaned out, then we can afford to wait for the
        // remainder of the timeout period to see if a fresh one comes in.
        int numRead = 0;
        do {
            //troll for a data packet, and if one is received then
            AsdDataPacket data = new AsdDataPacket(sock, timeout, type);
            if (data.read()) {
                //store it in the return list
                packets.add(data);
                ++numRead;
            }
            //while timeout hasn't expired
        } while(System.currentTimeMillis() - startTime < timeout);
        log_.debugf("ASD", "getLatestPacket pulled %d %s packets off the socket", numRead, type.toString());

        //return the latest message we've seen
        return packets;
    }

    private boolean					msgNeverParsed_;	//have we never parsed a MAP message during the life of this object?
    private int						prevVersion_;	    //content version of the previous MAP message we parsed
    private int						initTimeout_;
    private int						operTimeout_;
    private int						port_;		        //UDP port to listen for messages
    private DatagramSocket			socket_;
    private static ILogger			log_ = LoggerManager.getLogger(UdpForwardConsumer.class);
    private AsdInitializer			initializer_;
    private AsdMessageType          msgType_;          // MAP or SPAT
}
