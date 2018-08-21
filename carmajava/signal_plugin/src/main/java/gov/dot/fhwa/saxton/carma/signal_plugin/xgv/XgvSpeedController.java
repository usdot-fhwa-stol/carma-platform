package gov.dot.fhwa.saxton.carma.signal_plugin.xgv;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportComponentControlMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportDiscreteDevicesMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.ReportStatusMessage;

/**
 * Abstracts away the information needed to control the XGV Motion Profile Driver component
 */
public class XgvSpeedController {
    private XgvConnection connection;
    private ILogger log;

    public XgvSpeedController(XgvConnection connection) {
        this.connection = connection;
        log = LoggerManager.getLogger(getClass());
    }

    /**
     * Constructs and sends a Motion Profile message consisting of one motion frame for 0.1 seconds that instructs the
     * XGV to attempt to attain the proper speed. Will first query the control state of the MPD and attempt to gain
     * control of it if we do not already have it.
     *
     * If control is unable to be obtained an exception will be thrown, this is not necessarily a fatal error and a
     * reattempt may be made at the next timestep.
     *
     * @param targetSpeed A double value of the target speed in meters per second
     * @throws Exception if control of the MPD cannot be attained
     */
    public void sendMotionProfile(double targetSpeed) throws Exception {
        GlidepathAppConfig config = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig());
        log.debugf("", "Attempting to send motion profile with target speed %f", targetSpeed);

        // Query current controller of MPD
        log.info("", "Querying control status of the XGV Motion Profile Driver.");
        connection.sendQueryComponentControl(config.getMpdJausId());
        ReportComponentControlMessage m = new ReportComponentControlMessage(connection.waitForMessage(JausMessage.JausCommandCode.REPORT_COMPONENT_CONTROL));

        if (m.getCurrentControllerJausId() != config.getSoftwareJausId()) {
            // Request control ourselves
            log.debugf("", "Current controller of MPD = %d with authority", m.getCurrentControllerJausId(), m.getCurrentControllerAuthority());
            log.info("", "We are currently not in control of the motion profile driver. Requesting control of the Motion Profile Driver component with authority 255.");
            connection.sendRequestComponentControl(config.getMpdJausId(), 255);

            // send a resume to change status from standby to ready, has no affect if not in standby
            connection.sendEmptyPayloadMessage(config.getMpdJausId(), JausMessage.JausCommandCode.RESUME);
            connection.sendEmptyPayloadMessage(config.getPdJausId(), JausMessage.JausCommandCode.RESUME);

            // Verify that it worked
            log.info("", "Re-verifying component control of Motion Profile Driver");
            connection.sendQueryComponentControl(config.getMpdJausId());
            ReportComponentControlMessage m2 = new ReportComponentControlMessage(connection.waitForMessage(JausMessage.JausCommandCode.REPORT_COMPONENT_CONTROL));
            if (m2.getCurrentControllerJausId() != config.getSoftwareJausId()) {
                // We were unable to get control of the MPD
                log.error("!!!", "Unable to control the XGV Motion Profile Driver, will not be able to send speed control commands." +
                        " MPD currently controlled by  " + m2.getCurrentControllerJausId());
                throw new Exception("Unable to get component control of the XGV Motion Profile Driver.");
            }
        }

        log.info("", "We currently have control of the XGV Motion Profile Driver");

        // Construct and send the message to the XGV
        if (config.getXgvMPAck()) {
            connection.setRequestAck(true);
        }
        connection.sendSetMotionProfile(targetSpeed);
        if (config.getXgvMPAck()) {
            connection.setRequestAck(false);
        }
        log.info("", "Sent Motion Profile command to XGV with speed " + targetSpeed);
    }



    public void dumpXgvStatus(int motionProfileId) throws Exception    {
        ////////////////////////////
        // XGV Queries and Responses
        ////////////////////////////

        connection.sendComponentStatusQueryMessage(motionProfileId);
        JausMessage status = connection.waitForMessage(JausMessage.JausCommandCode.REPORT_COMPONENT_STATUS);

        connection.sendDiscreteDeviceStateQuery();
        JausMessage discreteDevices = connection.waitForMessage(JausMessage.JausCommandCode.REPORT_DISCRETE_DEVICES);

        //////////////////////////////////////////
        // Populate DataHolder fields and log data
        /////////////////////////////////////////

        if (status != null && discreteDevices != null) {
            XgvStatus xgvStatus = new XgvStatus(new ReportStatusMessage(status),
                    new ReportDiscreteDevicesMessage(discreteDevices));

            // Log contents of received message
            log.info("", "--- XGV Status Dump ---");
            log.info("", xgvStatus.toString());
        } else {
            log.warn("", "Failed to retrieve component status and/or discrete device status from XGV");
        }
    }
}
