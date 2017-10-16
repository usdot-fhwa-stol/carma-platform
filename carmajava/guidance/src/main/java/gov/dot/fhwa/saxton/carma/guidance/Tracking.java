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

import cav_msgs.AccelerationSet4Way;
import cav_msgs.BSM;
import cav_msgs.BSMCoreData;
import cav_msgs.HeadingStamped;
import cav_msgs.SystemAlert;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;
import nav_msgs.Odometry;

import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.RawMessage;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

import sensor_msgs.NavSatFix;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Tracking component
 * <p>
 * Reponsible for detecting when the vehicle strays from it's intended route or
 * trajectory and signalling the failure on the /system_alert topic
 */
public class Tracking extends GuidanceComponent {
    // Member variables
    protected final long sleepDurationMillis = 5000; // Not the real frequency for J2735
    protected int msgCount = 0;
    protected boolean drivers_ready = false;
    private IPublisher<SystemAlert> statusPublisher;
    private IPublisher<BSM> bsmPublisher;
    private ISubscriber<NavSatFix> navSatFixSubscriber;
    private ISubscriber<HeadingStamped> headingStampedSubscriber;
    private ISubscriber<TwistStamped> velocitySubscriber;
    private ISubscriber<Odometry> odometrySubscriber;
    private ISubscriber<AccelerationSet4Way> accelerationSubscriber;
    private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversService;
    private List<String> req_drivers = Arrays.asList(
    		"acceleration", "brake_position", "parking_brake", "speed", "steering_wheel_angle"
    		);
    private List<String> resp_drivers = new ArrayList();
    
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
        navSatFixSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
            @Override public void onMessage(NavSatFix msg) {
                log.info("Received NavSatFix:" + msg);
            }
        });

        headingStampedSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);
        headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
            @Override public void onMessage(HeadingStamped msg) {
                log.info("Received HeadingStamped:" + msg.toString());
            }
        });

        velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
        velocitySubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override public void onMessage(TwistStamped msg) {
                log.info("Received TwistStamped:" + msg.toString());
            }
        });
        
        odometrySubscriber = pubSubService.getSubscriberForTopic("odometry", Odometry._TYPE);
        odometrySubscriber.registerOnMessageCallback(new OnMessageCallback<Odometry>() {
            @Override public void onMessage(Odometry msg) {
                log.info("Received Odometry:" + msg.toString());
            }
        });
        
        // TODO: Integrate CAN data from Environment layer when available
        try {
			getDriversService = pubSubService.getServiceForTopic("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
			GetDriversWithCapabilitiesRequest driver_request_wrapper = getDriversService.newMessage();
			driver_request_wrapper.setCapabilities(req_drivers);
			getDriversService.callSync(driver_request_wrapper, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
				
				@Override
				public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
					resp_drivers = msg.getDriverData();
					log.info("call is successful!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" + resp_drivers.size());
				}
				
				@Override
				public void onFailure(Exception e) {
					throw new RosRuntimeException(e);
				}
			});
		} catch (Exception e) {
			handleException(e);
		}
        
        try {
        	accelerationSubscriber = pubSubService.getSubscriberForTopic(resp_drivers.get(0), AccelerationSet4Way._TYPE);
            accelerationSubscriber.registerOnMessageCallback(new OnMessageCallback<AccelerationSet4Way>() {
            	@Override public void onMessage(AccelerationSet4Way msg) {
                    log.info("Received accelerationSet:" + msg.toString());
                }
    		});
        } catch (Exception e) {
        	handleException(e);
		}
        
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

            //Publish dynamic BSM data
            bsmPublisher.publish(composeBSMData());
            
            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
            	handleException(e);
            }
    }
    
    private BSM composeBSMData() {
    	BSM bsmFrame = bsmPublisher.newMessage();
    	BSMCoreData coreData = bsmFrame.getCoreData();
        coreData.setMsgCount((byte) (msgCount++ % 127));
        
        //ID is fixed to identify each vehicle for now
        coreData.setId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, new byte[] {0, 0, 0, 1}));
        
        coreData.setSecMark((short) (System.currentTimeMillis() % 65535));
        coreData.setLatitude(navSatFixSubscriber.getLastMessage().getLatitude());
        coreData.setLongitude(navSatFixSubscriber.getLastMessage().getLongitude());
        coreData.setElev((float) navSatFixSubscriber.getLastMessage().getAltitude());
        
        //Left blank for now
        coreData.getAccuracy().setSemiMajor((float) 0);
        coreData.getAccuracy().setSemiMinor((float) 0);
        coreData.getAccuracy().setOrientation(0);
        
        coreData.getTransmission().setTransmissionState((byte) 2);
        
        coreData.setSpeed(0);
        coreData.setHeading((float) (headingStampedSubscriber.getLastMessage().getHeading() * 0.0125));
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
    	return bsmFrame;
    }
    
    protected void handleException(Exception e) {
		log.error(this.getComponentName() + "throws an exception and is about to shutdown...");
		node.shutdown();
	}
}
