/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

//TODO: Naming convention of "package gov.dot.fhwa.saxton.carmajava.<template>;"
//Originally "com.github.rosjava.carmajava.template;"
package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.BSM;
import cav_msgs.BSMCoreData;
import cav_msgs.HeadingStamped;
import cav_msgs.SystemAlert;
import com.sun.org.apache.bcel.internal.generic.ISUB;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import sensor_msgs.NavSatFix;

/**
 * Guidance package Tracking component
 * <p>
 * Reponsible for detecting when the vehicle strays from it's intended route or
 * trajectory and signalling the failure on the /system_alert topic
 */
public class Tracking implements Runnable {
    // Member variables
    protected final String componentName = "Tracking";
    protected final long sleepDurationMillis = 30000;
    protected ConnectedNode node;
    protected IPubSubService iPubSubService;
    protected int sequenceNumber = 0;
    protected Log log;
    protected int msgCount = 0;


    public Tracking(IPubSubService iPubSubService, ConnectedNode node) {
        this.iPubSubService = iPubSubService;
        this.node = node;
        this.log = node.getLog();
    }

    @Override public void run() {
        // Configure publishers for system status and BSM data
        IPublisher<SystemAlert> statusPublisher =
            iPubSubService.getPublisherForTopic("system_alert", cav_msgs.SystemAlert._TYPE);
        IPublisher<BSM> bsmPublisher = iPubSubService.getPublisherForTopic("bsm", BSM._TYPE);

        // Configure subscribers
        // TODO: Gather trajectory data internally from Guidance.Arbitrator and Guidance.Trajectory
        // TODO: Update when NavSatFix.msg is available
        ISubscriber<NavSatFix> navSatFixSubscriber = iPubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
            @Override public void onMessage(NavSatFix msg) {
                log.info("Received NavSatFix:" + msg);
            }
        });

        ISubscriber<HeadingStamped> headingStampedSubscriber = iPubSubService.getSubscriberForTopic(
            "heading", HeadingStamped._TYPE);

        headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
            @Override public void onMessage(HeadingStamped msg) {
                log.info("Received HeadingStamped:" + msg.toString());
            }
        });

        ISubscriber<TwistStamped> twistStampedSubscriber = iPubSubService.getSubscriberForTopic(
            "velocity", TwistStamped._TYPE);

        twistStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override public void onMessage(TwistStamped msg) {
                log.info("Received TwistStamped:" + msg.toString());
            }
        });

        // TODO: Integrate CAN data from Environment layer when available

        for (; ; ) {
            cav_msgs.SystemAlert systemAlertMsg = statusPublisher.newMessage();
            systemAlertMsg
                .setDescription("Tracking has not detected a running trajectory, no means to compute"
                    + " crosstrack error");
            systemAlertMsg.setType(SystemAlert.CAUTION);
            statusPublisher.publish(systemAlertMsg);

            // Generate static BSM data
            cav_msgs.BSM bsmFrame = bsmPublisher.newMessage();
            BSMCoreData coreData = bsmFrame.getCoreData();
            coreData.setId((byte) 0x80);
            coreData.setMsgCount((byte) (msgCount++ % 255));
            coreData.setSecMark((short) ((node.getCurrentTime().nsecs / 1000) % 65536));
            coreData.setAngle(0);
            coreData.getAccelSet().setLatitude(0);
            coreData.getAccelSet().setLongitude(0);
            coreData.getAccelSet().setVert(0);
            coreData.getAccelSet().setYaw(0);
            coreData.getBrakes().getAbs().setAntiLockBrakeStatus((byte) 0);
            coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus((byte) 0);
            coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied((byte) 0);
            coreData.getBrakes().getScs().setStabilityControlStatus((byte) 0);
            coreData.getBrakes().getTraction().setTractionControlStatus((byte) 0);
            coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 0);
            coreData.getAccuracy().setOrientation(0);
            coreData.getAccuracy().setSemiMajor(0);
            coreData.getAccuracy().setSemiMinor(0);
            coreData.setElev(0);
            coreData.setHeading(0);
            coreData.setLatitude(38.956474);
            coreData.setLongitude(-77.150279);
            coreData.getSize().setVehicleLength(6);
            coreData.getSize().setVehicleWidth(3);
            coreData.setSpeed(0);
            coreData.getTransmission().setTransmissionState((byte) 0 );

            // Publish the BSM data
            bsmPublisher.publish(bsmFrame);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
            }
        }
    }
}
