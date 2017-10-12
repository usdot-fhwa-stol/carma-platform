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
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.node.ConnectedNode;
import sensor_msgs.NavSatFix;

import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Tracking component
 * <p>
 * Reponsible for detecting when the vehicle strays from it's intended route or
 * trajectory and signalling the failure on the /system_alert topic
 */
public class Tracking extends GuidanceComponent {
    // Member variables
    protected final long sleepDurationMillis = 30000;
    protected int msgCount = 0;
    private IPublisher<SystemAlert> statusPublisher;
    private ISubscriber<NavSatFix> navSatFixSubscriber;
    private ISubscriber<HeadingStamped> headingStampedSubscriber;
    private ISubscriber<TwistStamped> twistStampedSubscriber;
    private IPublisher<BSM> bsmPublisher;

    public Tracking(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
        super(state, pubSubService, node);
    }

    @Override public String getComponentName() {
        return "Guidance.Tracking";
    }

    @Override public void onGuidanceStartup() {
        statusPublisher =
            pubSubService.getPublisherForTopic("system_alert", cav_msgs.SystemAlert._TYPE);
        bsmPublisher = pubSubService.getPublisherForTopic("bsm", BSM._TYPE);

        // Configure subscribers
        // TODO: Gather trajectory data internally from Guidance.Arbitrator and Guidance.Trajectory
        // TODO: Update when NavSatFix.msg is available
        navSatFixSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
            @Override public void onMessage(NavSatFix msg) {
                log.info("Received NavSatFix:" + msg);
            }
        });

        headingStampedSubscriber = pubSubService.getSubscriberForTopic(
            "heading", HeadingStamped._TYPE);

        headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
            @Override public void onMessage(HeadingStamped msg) {
                log.info("Received HeadingStamped:" + msg.toString());
            }
        });

        twistStampedSubscriber = pubSubService.getSubscriberForTopic(
            "velocity", TwistStamped._TYPE);

        twistStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override public void onMessage(TwistStamped msg) {
                log.info("Received TwistStamped:" + msg.toString());
            }
        });

        // TODO: Integrate CAN data from Environment layer when available
    }

    @Override public void onSystemReady() {
        // NO-OP
    }

    @Override public void onGuidanceEnable() {

    }

    @Override public void loop() {
            cav_msgs.SystemAlert systemAlertMsg = statusPublisher.newMessage();
            systemAlertMsg
                .setDescription("Tracking has not detected a running trajectory, no means to compute"
                    + " crosstrack error");
            systemAlertMsg.setType(SystemAlert.CAUTION);
            statusPublisher.publish(systemAlertMsg);

            // Generate static BSM data
            cav_msgs.BSM bsmFrame = bsmPublisher.newMessage();
            BSMCoreData coreData = bsmFrame.getCoreData();
            coreData.setMsgCount((byte) 101);
            coreData.setId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, new byte[] {(byte) 0xAB, 0x44, (byte) 0xD4, 0x4D}));
            coreData.setSecMark((short) 31202);
            coreData.setLatitude(41252 / 10000000.0);
            coreData.setLongitude(-21000001 / 10000000.0);
            coreData.setElev((float) (312 / 10.0));
            coreData.getAccuracy().setSemiMajor((float) (145 * 0.05));
            coreData.getAccuracy().setSemiMinor((float) (125 * 0.05));
            coreData.getAccuracy().setOrientation(30252 * 0.054932479);
            coreData.getTransmission().setTransmissionState((byte) 2);
            coreData.setSpeed((float) (2100 * 0.02));
            coreData.setHeading((float) (22049 * 0.0125));
            coreData.setAngle((float) (13 * 1.5));
            coreData.getAccelSet().setLongitude((float) (12 * 0.01));
            coreData.getAccelSet().setLatitude((float) (-180 * 0.01));
            coreData.getAccelSet().setVert((float) (55 * 0.02));
            coreData.getAccelSet().setYaw((float) (-16001 * 0.01));
            coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 0x48); //TODO change it to correct description
            coreData.getBrakes().getTraction().setTractionControlStatus((byte) 2);
            coreData.getBrakes().getAbs().setAntiLockBrakeStatus((byte) 3);
            coreData.getBrakes().getScs().setStabilityControlStatus((byte) 1);
            coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied((byte) 0);
            coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus((byte) 1);
            coreData.getSize().setVehicleWidth((float) (199 / 100.0));
            coreData.getSize().setVehicleLength((float) (3069 / 100.0));

            // Publish the BSM data
            bsmPublisher.publish(bsmFrame);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
            }
    }
}
