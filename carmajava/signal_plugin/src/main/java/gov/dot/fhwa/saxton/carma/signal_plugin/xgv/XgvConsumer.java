package gov.dot.fhwa.saxton.carma.signal_plugin.xgv;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;
import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerTask;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.*;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportDiscreteDevicesMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportStatusMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportVelocityMessage;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

/**
 * Queries the XGV for status and speed data.
 */
public class XgvConsumer implements IConsumerTask {

    private XgvInitializer initializer;
    private XgvConnection connection;
    private ILogger log;

    public XgvConsumer() {
        this.log = LoggerManager.getLogger(getClass());
    }

    @Override
    public boolean initialize() {
        // Initialize the XgvConnection and its sockets
        int judpPort = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getJausUdpPort();
        String xgvIpAddress = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getXgvIpAddress();

        log.info("XGV", "Opening inbound datagram socket on port " + judpPort);
        DatagramSocket sock = null;
        try {
            sock = new DatagramSocket(judpPort); //this socket gets closed by the XgvConnection object to which it is passed

	        if (sock != null) {
	            InetSocketAddress outbound = new InetSocketAddress(xgvIpAddress, judpPort);
	            if (outbound != null) {
	            	log.debugf("XGV", "outbound address created: address = %s, port = %d", outbound.getAddress().toString(), outbound.getPort());
	            }
	            connection = new XgvConnection(sock, outbound);
	            log.info("XGV", "XgvConnection successfully created.");
	            return true;
	        }
	        return false;
	        
        } catch (SocketException e) {
            log.errorf("XGV", "Sockets for XgvConnection failed to open! %s", e.toString());
            return false;
        }
    }

    @Override
    public void terminate() {
        releaseMpdControl();
        connection.close();
    }

    @Override
    public IConsumerInitializer getInitializer() {
    	log.debug("XGV", "XgvConsumer creating a new XgvInitializer");
        initializer = new XgvInitializer(connection);
        return initializer;
    }

    public void releaseMpdControl()   {
        int mpd = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getMpdJausId();

        try   {
            // put in standby
            connection.sendEmptyPayloadMessage(mpd, JausMessage.JausCommandCode.STANDBY);

            // send RELEASE COMPONENT CONTROL message to MPD
            connection.sendReleaseComponentControl(mpd);

        }
        catch(Exception e)   {
            log.error("XGV", "Error attempting to release MPD control: " + e.getMessage());
        }
    }


    @Override
    public DataElementHolder call() throws Exception {
        // for now, record duration of call
        DateTime startTime = new DateTime();

        DataElementHolder holder = new DataElementHolder();

        ////////////////////////////
        // XGV Queries and Responses
        ////////////////////////////

        int motionProfileId = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getMpdJausId();
        connection.sendComponentStatusQueryMessage(motionProfileId);
        DateTime time1 = new DateTime();
        JausMessage status = connection.waitForMessage(JausMessage.JausCommandCode.REPORT_COMPONENT_STATUS);
        DateTime time2 = new DateTime();

        connection.sendQueryVelocityStateMessage();
        DateTime time3 = new DateTime();
        JausMessage velocity = connection.waitForMessage(JausMessage.JausCommandCode.REPORT_VELOCITY_STATE);
        DateTime time4 = new DateTime();

        connection.sendDiscreteDeviceStateQuery();
        DateTime time5 = new DateTime();
        JausMessage discreteDevices = connection.waitForMessage(JausMessage.JausCommandCode.REPORT_DISCRETE_DEVICES);
        DateTime time6 = new DateTime();
        
        log.debug("XGV", "Timing breakdown for consumer call():");
        log.debugf("XGV", "    sendComponentStatusQueryMessage took %3d ms, response took %3d ms",
        			new Duration(startTime, time1).getMillis(), new Duration(time1, time2).getMillis());
        log.debugf("XGV", "    sendQueryVelocityStateMessage took   %3d ms, response took %3d ms",
        			new Duration(time2, time3).getMillis(), new Duration(time3, time4).getMillis());
        log.debugf("XGV", "    sendDiscreteDeviceStateQuery took    %3d ms, response took %3d ms", 
        			new Duration(time4, time5).getMillis(), new Duration(time5, time6).getMillis());
        log.debugf("XGV", "    Total query/response time = %d ms", new Duration(startTime, time6).getMillis());

        //////////////////////////////////////////
        // Populate DataHolder fields and log data
        /////////////////////////////////////////
        
        if (status != null && discreteDevices != null) {
            XgvStatus xgvStatus = new XgvStatus(new ReportStatusMessage(status),
                                                new ReportDiscreteDevicesMessage(discreteDevices));
            XgvStatusDataElement statusDataElement = new XgvStatusDataElement(xgvStatus);
            holder.put(DataElementKey.XGV_STATUS, statusDataElement);

            // Log contents of received message
            log.info("", "--- XGV Status Dump ---");
            log.info("", xgvStatus.toString());
        } else {
            log.warn("", "Failed to retrieve component status and/or discrete device status from XGV");
        }

        if (velocity != null) {
            ReportVelocityMessage velocityMessage = new ReportVelocityMessage(velocity);
            DoubleDataElement velocityDataElement = new DoubleDataElement(velocityMessage.getVelocity());
            holder.put(DataElementKey.SPEED, velocityDataElement);
            log.info("", "Current velocity as reported by XGV: " + velocityMessage.getVelocity());
        } else {
            log.warn("", "Failed to retrieve velocity state from XGV");
        }

        Duration duration = new Duration(startTime, new DateTime());
        holder.put(DataElementKey.CYCLE_XGV, new IntDataElement((int) duration.getMillis()));
        log.debug("XGV", "XGV Consumer call() millis: " + duration.getMillis());

        return holder;
    }

    /**
     * Allows for external access to this object's XgvConnection instance, primarily for construction of an XgvSpeedController.
     * @return The XgvConnection instance associated with this consumer.
     */
    public XgvConnection getConnection() {
        return connection;
    }
}
