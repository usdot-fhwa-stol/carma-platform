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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.*;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;

import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import sensor_msgs.NavSatFix;
import std_msgs.Float64;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Tracking component
 * <p>
 * Reponsible for detecting when the vehicle strays from it's intended route or
 * trajectory and signalling the failure on the /system_alert topic
 */
public class Tracking extends GuidanceComponent {
	// Member variables
	protected final long sleepDurationMillis = 100; // Frequency for J2735
	private int msgCount = 0;
	private float vehicleWidth = 0;
	private float vehicleLength = 0;
	private boolean drivers_ready = false;
	private boolean steer_wheel_ready = false;
	private boolean nav_sat_fix_ready = false;
	private boolean heading_ready = false;
	private boolean velocity_ready = false;
	protected GuidanceExceptionHandler exceptionHandler;
	private Random randomIdGenerator = new Random();
	private byte[] random_id = new byte[4];
	private IPublisher<BSM> bsmPublisher;
	private ISubscriber<NavSatFix> navSatFixSubscriber;
	private ISubscriber<HeadingStamped> headingStampedSubscriber;
	private ISubscriber<TwistStamped> velocitySubscriber;
	private ISubscriber<Float64> steeringWheelSubscriber;
	private ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient;
	private List<String> req_drivers = Arrays.asList("steering_wheel_angle");
	private List<String> resp_drivers;

	public Tracking(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
		super(state, pubSubService, node);
		this.exceptionHandler = new GuidanceExceptionHandler(state, log);
	}

	@Override
	public String getComponentName() {
		return "Guidance.Tracking";
	}

	@Override
	public void onGuidanceStartup() {
		
		// Publishers
		bsmPublisher = pubSubService.getPublisherForTopic("bsm", BSM._TYPE);

		// Subscribers
		navSatFixSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
		navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
			@Override
			public void onMessage(NavSatFix msg) {
				nav_sat_fix_ready = true;
			}
		});
		
		headingStampedSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);
		headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
			@Override
			public void onMessage(HeadingStamped msg) {
				heading_ready = true;
			}
		});
		
		velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
		velocitySubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
			@Override
			public void onMessage(TwistStamped msg) {
				velocity_ready = true;
			}
		});
	}

	@Override
	public void onSystemReady() {
		
		// Make service call to get drivers
		// Do not have access to SaxtonBaseNode method from here. Implement a simple waitForService.
		try {
			log.info("Tracking is trying to get get_drivers_with_capabilities service...");
			int counter = 0;
			while(getDriversWithCapabilitiesClient == null && counter++ < 10) {
				try{
					getDriversWithCapabilitiesClient = node.newServiceClient("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
				} catch (ServiceNotFoundException ex_0) {
					log.warn("Tracking: node could not find service get_drivers_with_capabilities.");
				} catch (RosRuntimeException ex_1) {
					log.info("Tracking: runtime exception happened and ignored.");
				}
				if(getDriversWithCapabilitiesClient == null) {
					log.warn("Tracking: Trying to find service get_drivers_with_capabilities again.");
				}
				Thread.sleep(1000);
			}
			if(getDriversWithCapabilitiesClient == null) {
				throw new ServiceNotFoundException("get_drivers_with_capabilities service can not be found");
			}
			log.info("Tracking is making a service call to interfaceMgr...");
			GetDriversWithCapabilitiesRequest driver_request_wrapper = getDriversWithCapabilitiesClient.newMessage();
			driver_request_wrapper.setCapabilities(req_drivers);
			RosServiceSynchronizer.callSync(getDriversWithCapabilitiesClient, driver_request_wrapper, new ServiceResponseListener<GetDriversWithCapabilitiesResponse>() {
				
				@Override
				public void onFailure(RemoteException arg0) {
					throw new RosRuntimeException(arg0);
				}

				@Override
				public void onSuccess(GetDriversWithCapabilitiesResponse arg0) {
					resp_drivers = arg0.getDriverData();
					log.info("Tracking: service call is successful: " + resp_drivers);
				}
				
			});
		} catch (Exception e) {
			handleException(e);
		}
		
		steeringWheelSubscriber = pubSubService.getSubscriberForTopic(resp_drivers.get(0), Float64._TYPE);
		steeringWheelSubscriber.registerOnMessageCallback(new OnMessageCallback<Float64>() {
			@Override
			public void onMessage(Float64 msg) {
				steer_wheel_ready = true;
			}
		});

		drivers_ready = true;
	}

	@Override
	public void onGuidanceEnable() {
	}

	@Override
	public void loop() throws InterruptedException {
		if(drivers_ready) {
			try {
				log.info("Tracking subscribers status: " + nav_sat_fix_ready + " " + steer_wheel_ready + " " + heading_ready + " " + velocity_ready);
				if (nav_sat_fix_ready && steer_wheel_ready && heading_ready && velocity_ready) {
					log.info("Guidance.Tracking is publishing bsm...");
					bsmPublisher.publish(composeBSMData());
				}
			}  catch (Exception e) {
				handleException(e);
			}
		}
		Thread.sleep(sleepDurationMillis);
	}

	private BSM composeBSMData() {

		BSM bsmFrame = bsmPublisher.newMessage();

		try {
			if (msgCount == 0) {
				randomIdGenerator.nextBytes(random_id);
			}

			// Set header
			bsmFrame.getHeader().setStamp(node.getCurrentTime());
			bsmFrame.getHeader().setFrameId("MessageNode");

			// Set core data
			BSMCoreData coreData = bsmFrame.getCoreData();
			coreData.setMsgCount((byte) (msgCount++ % 127));

			// ID is random and changes every 5 minutes
			if (msgCount == 3000) {
				msgCount = 0;
			}
			coreData.setId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, random_id));

			// Use ros node time
			coreData.setSecMark((short) (node.getCurrentTime().toSeconds() % 65535));

			// Set GPS data
			coreData.setLatitude(navSatFixSubscriber.getLastMessage().getLatitude());
			coreData.setLongitude(navSatFixSubscriber.getLastMessage().getLongitude());
			coreData.setElev((float) navSatFixSubscriber.getLastMessage().getAltitude());

			// N/A for now
			coreData.getAccuracy().setSemiMajor((float) (255 * 0.05));
			coreData.getAccuracy().setSemiMinor((float) (255 * 0.05));
			coreData.getAccuracy().setOrientation(65535 * 0.0054932479);
			coreData.getTransmission().setTransmissionState((byte) 7);

			coreData.setSpeed((float) velocitySubscriber.getLastMessage().getTwist().getLinear().getX());
			coreData.setHeading((float) (headingStampedSubscriber.getLastMessage().getHeading()));
			coreData.setAngle((float) steeringWheelSubscriber.getLastMessage().getData());

			// N/A for now
			coreData.getAccelSet().setLongitudinal((float) (2001 * 0.01));
			coreData.getAccelSet().setLateral((float) (2001 * 0.01));
			coreData.getAccelSet().setVert((float) (-127 * 0.02));
			coreData.getAccelSet().setYawRate(0);
			coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 10);
			coreData.getBrakes().getTraction().setTractionControlStatus((byte) 0);
			coreData.getBrakes().getAbs().setAntiLockBrakeStatus((byte) 0);
			coreData.getBrakes().getScs().setStabilityControlStatus((byte) 0);
			coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied((byte) 0);
			coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus((byte) 0);

			// Set length and width only for the first time
			if (vehicleLength == 0 && vehicleWidth == 0) {
				ParameterTree param = node.getParameterTree();
				vehicleLength = (float) param.getDouble("~vehicle_length");
				vehicleWidth = (float) param.getDouble("~vehicle_width");
			}
			coreData.getSize().setVehicleLength(vehicleLength);
			coreData.getSize().setVehicleWidth(vehicleWidth);

		} catch (Exception e) {
			handleException(e);
		}
		return bsmFrame;
	}

	protected void handleException(Exception e) {
		log.error("Tracking throws an exception...");
		exceptionHandler.handleException(e);
	}
}
