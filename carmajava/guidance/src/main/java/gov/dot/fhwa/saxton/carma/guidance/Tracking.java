/*
 * Copyright (C) 2017 LEIDOS.
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
import cav_srvs.GetTransform;
import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import geometry_msgs.AccelStamped;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

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
	protected static final float VERTICAL_ACCELERATION_UNAVAILABLE = -24.892f;
	protected static final float ACCELERATION_UNAVAILABLE = 20.01f;
	protected static final int STEEL_WHEEL_ANGLE_MAX = 189;
	protected static final int STEEL_WHEEL_ANGLE_MIN = -189;
	protected static final float STEEL_WHEEL_ANGLE_UNAVAILABLE = 190.5f;
	protected static final float HEADING_MAX = 359.9875f;
	protected static final int HEADING_UNAVAILABLE = 360;
	protected static final float SPEED_MAX = 163.8f;
	protected static final float SPEED_UNAVAILABLE = 163.82f;
	protected static final double ACCURACY_ORIENTATION_MAX = 359.9945078786;
	protected static final float ACCURACY_MAX = 12.7f;
	protected static final float ELEVATION_MAX = 6143.9f;
	protected static final int LONGITUDE_MAX = 180;
	protected static final float ELEVATION_MIN = -409.5f;
	protected static final double LONGITUDE_MIN = -179.9999999;
	protected static final int LATITUDE_MAX = 90;
	protected static final int LATITUDE_MIN = -90;
	protected static final double ACCURACY_ORIENTATION_UNAVAILABLE = 65535 * 0.0054932479;
	protected static final float ACCURACY_UNAVAILABLE = 12.75f;
	protected static final float ELEVATION_UNAVAILABLE = 409.6f;
	protected static final double LONGITUDE_UNAVAILABLE = 180.0000001;
	protected static final double LATITUDE_UNAVAILABLE = 90.0000001;
	protected static final int DSECOND_MAX = 65535;
	protected static final int ID_TIME_MAX = 3000;
	protected static final int MSG_COUNT_MAX = 127; // 10 msg/sec * 5 min * 60 sec/min
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
	private boolean brake_ready = false;
	private boolean transmission_ready = false;
	private boolean acceleration_ready = false;
	private boolean vehicle_to_earth_transform_ready = false;
	private boolean base_to_map_transform_ready = false;
	private boolean earth_to_map_ready = false;
	protected GuidanceExceptionHandler exceptionHandler;
	protected SaxtonLogger log_ = new SaxtonLogger(Tracking.class.getSimpleName(), log);
	private Random randomIdGenerator = new Random();
	private byte[] random_id = new byte[4];
	private IPublisher<BSM> bsmPublisher;
	private ISubscriber<AccelStamped> accelerationSubscriber;
	private ISubscriber<NavSatFix> navSatFixSubscriber;
	private ISubscriber<HeadingStamped> headingStampedSubscriber;
	private ISubscriber<TwistStamped> velocitySubscriber;
	private ISubscriber<Float64> steeringWheelSubscriber;
	private ISubscriber<Float64> brakeSubscriber;
	private ISubscriber<TransmissionState> transmissionSubscriber;
	private IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient;
	private IService<GetTransformRequest, GetTransformResponse> getTransformClient;
	private List<String> req_drivers = Arrays.asList("steering_wheel_angle", "brake_position", "transmission_state");
	private List<String> resp_drivers;
	private final String baseLinkFrame = "base_link";
	private final String vehicleFrame = "host_vehicle";
	private final String mapFrame = "map";
	private final String earthFrame = "earth";
	private Transform vehicleToEarth = null;
	private Transform earthtoMap = null;
	private Transform baseToMap = null;
	private GeodesicCartesianConverter converter = new GeodesicCartesianConverter();
	

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
		
		try {
			// Publishers
			bsmPublisher = pubSubService.getPublisherForTopic("bsm", BSM._TYPE);

			// Subscribers
			navSatFixSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
			headingStampedSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);
			velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
			accelerationSubscriber = pubSubService.getSubscriberForTopic("acceleration", AccelerationSet4Way._TYPE);
			
			if(bsmPublisher == null 
					|| navSatFixSubscriber == null 
					|| headingStampedSubscriber == null 
					|| velocitySubscriber == null 
					|| accelerationSubscriber == null) {
				log_.warn("Cannot initialize pubs and subs");
			}
			
			navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
				@Override
				public void onMessage(NavSatFix msg) {
					nav_sat_fix_ready = true;
				}
			});
			
			
			headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
				@Override
				public void onMessage(HeadingStamped msg) {
					heading_ready = true;
				}
			});
			
			
			velocitySubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
				@Override
				public void onMessage(TwistStamped msg) {
					velocity_ready = true;
				}
			});
			
			accelerationSubscriber.registerOnMessageCallback(new OnMessageCallback<AccelStamped>() {
				@Override
				public void onMessage(AccelStamped msg) {
					acceleration_ready = true;
				}
			});
			
		} catch (Exception e) {
			handleException(e);
		}
		
	}

	@Override
	public void onSystemReady() {
		
		// Make service call to get drivers
		try {
			log_.info("Trying to get get_drivers_with_capabilities service...");
			getDriversWithCapabilitiesClient = pubSubService.getServiceForTopic("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
			if(getDriversWithCapabilitiesClient == null) {
				log_.warn("get_drivers_with_capabilities service can not be found");
			}
			
			GetDriversWithCapabilitiesRequest driver_request_wrapper = getDriversWithCapabilitiesClient.newMessage();
			driver_request_wrapper.setCapabilities(req_drivers);
			getDriversWithCapabilitiesClient.callSync(driver_request_wrapper, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
				
				@Override
				public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
					resp_drivers = msg.getDriverData();
					log_.info("Tracking: service call is successful: " + resp_drivers);
				}
				
				@Override
				public void onFailure(Exception e) {
					throw new RosRuntimeException(e);
				}
				
			});
			
			log_.info("Trying to get get_transform...");
			getTransformClient = pubSubService.getServiceForTopic("get_transform", GetTransform._TYPE);
			if(getTransformClient == null) {
				log_.warn("get_transform service can not be found");
			}
			
			if(resp_drivers != null) {
				for(String driver_url : resp_drivers) {
					if(driver_url.endsWith("/can/steering_wheel_angle")) {
						steeringWheelSubscriber = pubSubService.getSubscriberForTopic(driver_url, Float64._TYPE);
						continue;
					}
					if(driver_url.endsWith("/can/brake_position")) {
						brakeSubscriber = pubSubService.getSubscriberForTopic(driver_url, Float64._TYPE);
						continue;
					}
					if(driver_url.endsWith("/can/transmission_state")) {
						transmissionSubscriber = pubSubService.getSubscriberForTopic(driver_url, TransmissionState._TYPE);
					}
				}
			} else {
				log_.warn("Tracking: cannot find suitable drivers");
			}
			
			if(steeringWheelSubscriber == null || brakeSubscriber == null || transmissionSubscriber == null) {
				log_.warn("Tracking: initialize subs failed");
			}
			
			steeringWheelSubscriber.registerOnMessageCallback(new OnMessageCallback<Float64>() {
				@Override
				public void onMessage(Float64 msg) {
					steer_wheel_ready = true;
				}
			});
			
			brakeSubscriber.registerOnMessageCallback(new OnMessageCallback<Float64>() {
				@Override
				public void onMessage(Float64 msg) {
					brake_ready = true;
				}
			});
			
			transmissionSubscriber.registerOnMessageCallback(new OnMessageCallback<TransmissionState>() {
				@Override
				public void onMessage(TransmissionState msg) {
					transmission_ready = true;
				}
			});
			
		} catch (Exception e) {
			handleException(e);
		}
		
		
		
		ParameterTree param = node.getParameterTree();
		vehicleLength = (float) param.getDouble("~vehicle_length");
		vehicleWidth = (float) param.getDouble("~vehicle_width");
		
		drivers_ready = true;
	}

	@Override
	public void onGuidanceEnable() {
	}

	@Override
	public void loop() throws InterruptedException {
		
		if(drivers_ready) {
			try {
				log_.info("BSM", "nav_sat_fix subscribers status: " + nav_sat_fix_ready);
				log_.info("BSM", "steer_wheel subscribers status: " + steer_wheel_ready);
				log_.info("BSM", "heading subscribers status: " + heading_ready);
				log_.info("BSM", "velocity subscribers status: " + velocity_ready);
				log_.info("BSM", "brake subscribers status: " + brake_ready);
				log_.info("BSM", "transmission subscribers status: " + transmission_ready);
				log_.info("BSM", "acceleration subscribers status: " + acceleration_ready);
				log_.info("BSM", "Guidance.Tracking is publishing bsm...");
				bsmPublisher.publish(composeBSMData());
			} catch (Exception e) {
				handleException(e);
			}
		}
		
		Thread.sleep(sleepDurationMillis);
		
	}

	private BSM composeBSMData() {

		BSM bsmFrame = bsmPublisher.newMessage();

		try {
			// Set header
			bsmFrame.getHeader().setStamp(node.getCurrentTime());
			bsmFrame.getHeader().setFrameId(Tracking.class.getSimpleName());

			// Set core data
			BSMCoreData coreData = bsmFrame.getCoreData();
			coreData.setMsgCount((byte) (msgCount % MSG_COUNT_MAX));

			// ID is random and changes every 5 minutes
			if (msgCount == 0) {
				randomIdGenerator.nextBytes(random_id);
				msgCount++;
			} else if (msgCount == ID_TIME_MAX) {
				msgCount = 0;
			}
			coreData.setId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, random_id));

			// Set GPS data
			coreData.setLatitude(LATITUDE_UNAVAILABLE); // Default value when unknown
			coreData.setLongitude(LONGITUDE_UNAVAILABLE);
			coreData.setElev(ELEVATION_UNAVAILABLE);
			coreData.getAccuracy().setSemiMajor(ACCURACY_UNAVAILABLE);
			coreData.getAccuracy().setSemiMinor(ACCURACY_UNAVAILABLE);
			coreData.getAccuracy().setOrientation(ACCURACY_ORIENTATION_UNAVAILABLE);
			if(nav_sat_fix_ready && getTransformClient != null) {
				GetTransformRequest transform_request = getTransformClient.newMessage();
				transform_request.setParentFrame(earthFrame);
				transform_request.setChildFrame(vehicleFrame);
				getTransformClient.callSync(transform_request, new OnServiceResponseCallback<GetTransformResponse>() {
					
					@Override
					public void onSuccess(GetTransformResponse msg) {
						log_.info("BSM", "Get vehicle_to_earth_transform response " + msg.getErrorStatus());
						if(msg.getErrorStatus() == 0) {
							//TODO: fix this transform later
							vehicleToEarth = Transform.fromTransformMessage(msg.getTransform().getTransform());
							vehicle_to_earth_transform_ready = true;
						}
					}
					
					@Override
					public void onFailure(Exception e) {
						throw new RosRuntimeException(e);
					}
				});
				
				//TODO: find transformation
//				transform_request.setParentFrame(earthFrame);
//				transform_request.setChildFrame(vehicleFrame);
//				getTransformClient.callSync(transform_request, new OnServiceResponseCallback<GetTransformResponse>() {
//					
//					@Override
//					public void onSuccess(GetTransformResponse msg) {
//						log_.info("BSM", "Get vehicle_to_earth_transform response " + msg.getErrorStatus());
//						if(msg.getErrorStatus() == 0) {
//							//TODO: fix this transform later
//							vehicleToEarth = Transform.fromTransformMessage(msg.getTransform().getTransform());
//							vehicle_to_earth_transform_ready = true;
//						}
//					}
//					
//					@Override
//					public void onFailure(Exception e) {
//						throw new RosRuntimeException(e);
//					}
//				});
				
				if(vehicle_to_earth_transform_ready && earth_to_map_ready) {
					Point3D point_3d = new Point3D(
							vehicleToEarth.getTranslation().getX(), 
							vehicleToEarth.getTranslation().getY(), 
							vehicleToEarth.getTranslation().getZ()); 
					Location location = converter.cartesian2Geodesic(point_3d, earthtoMap);
					double lat = location.getLatitude();
					double Lon = location.getLongitude();
					float elev = (float) location.getAltitude();
					if(lat >= LATITUDE_MIN && lat <= LATITUDE_MAX) {
						coreData.setLatitude(lat);
					}
					if(Lon >= LONGITUDE_MIN && Lon <= LONGITUDE_MAX) {
						coreData.setLongitude(Lon);
					}
					if(elev >= ELEVATION_MIN && elev <= ELEVATION_MAX) {
						coreData.setElev(elev);
					}
				}
				
				float semi_major = (float) navSatFixSubscriber.getLastMessage().getPositionCovariance()[0];
				float semi_minor = (float) navSatFixSubscriber.getLastMessage().getPositionCovariance()[4];
				double orientation = 0; //orientation of semi_major axis
				
				if(semi_major >= 0) {
					if(Math.sqrt(semi_major) >= ACCURACY_MAX) {
						coreData.getAccuracy().setSemiMajor(ACCURACY_MAX);
					} else {
						coreData.getAccuracy().setSemiMajor((float) Math.sqrt(semi_major));
					}
					
				}
				
				if(semi_minor >= 0) {
					if(Math.sqrt(semi_minor) >= ACCURACY_MAX) {
						coreData.getAccuracy().setSemiMinor(ACCURACY_MAX);
					} else {
						coreData.getAccuracy().setSemiMinor((float) Math.sqrt(semi_minor));
					}
				}
				
				if(orientation >= 0 && orientation <= ACCURACY_ORIENTATION_MAX) {
					coreData.getAccuracy().setOrientation(orientation);
				}
			}
			
			// Set transmission state
			coreData.getTransmission().setTransmissionState(TransmissionState.UNAVAILABLE);
			if(transmission_ready) {
				TransmissionState transmission_state = transmissionSubscriber.getLastMessage();
				if(transmission_state.getTransmissionState() >= 0 && transmission_state.getTransmissionState() < TransmissionState.UNAVAILABLE) {
					coreData.getTransmission().setTransmissionState(transmission_state.getTransmissionState());
				}
			}

			coreData.setSpeed(SPEED_UNAVAILABLE);
			if(velocity_ready) {
				float speed = (float) velocitySubscriber.getLastMessage().getTwist().getLinear().getX();
				if(speed >= 0 && speed <= SPEED_MAX) {
					coreData.setSpeed(speed);
				}
			}
			
			coreData.setHeading(HEADING_UNAVAILABLE);
			if(heading_ready) {
				float heading = (float) headingStampedSubscriber.getLastMessage().getHeading();
				if(heading >= 0 && heading <= HEADING_MAX) {
					coreData.setHeading(heading);
				}
			}
			
			coreData.setAngle(STEEL_WHEEL_ANGLE_UNAVAILABLE);
			if(steer_wheel_ready) {
				
				float angle = (float) steeringWheelSubscriber.getLastMessage().getData();	
				if(angle <= STEEL_WHEEL_ANGLE_MIN) {
					coreData.setAngle(STEEL_WHEEL_ANGLE_MIN);
				} else if(angle >= STEEL_WHEEL_ANGLE_MAX) {
					coreData.setAngle(STEEL_WHEEL_ANGLE_MAX);
				} else {
					coreData.setAngle(angle);
				}
			}

			// N/A for now
			coreData.getAccelSet().setLongitudinal(ACCELERATION_UNAVAILABLE);
			coreData.getAccelSet().setLateral(ACCELERATION_UNAVAILABLE);
			coreData.getAccelSet().setVert(VERTICAL_ACCELERATION_UNAVAILABLE);
			coreData.getAccelSet().setYawRate(0);  //TODO: It is not defined well
			if(acceleration_ready && getTransformClient != null) {
				GetTransformRequest transform_request = getTransformClient.newMessage();
				transform_request.setParentFrame(mapFrame);
				transform_request.setChildFrame(baseLinkFrame);
				getTransformClient.callSync(transform_request, new OnServiceResponseCallback<GetTransformResponse>() {
					
					@Override
					public void onSuccess(GetTransformResponse msg) {
						log_.info("BSM", "Get base_to_map_transform response " + msg.getErrorStatus());
						if(msg.getErrorStatus() == 0) {
							baseToMap = Transform.fromTransformMessage(msg.getTransform().getTransform());
							base_to_map_transform_ready = true;
						}
					}
					
					@Override
					public void onFailure(Exception e) {
						throw new RosRuntimeException(e);
					}
				});
				
				if(base_to_map_transform_ready) {
					geometry_msgs.Vector3 accel_linear = accelerationSubscriber.getLastMessage().getAccel().getLinear();
					Vector3 after_transform_accel_linear = baseToMap.apply((Vector3) accel_linear);
					
					if(after_transform_accel_linear.getX() <= -20) {
						coreData.getAccelSet().setLongitudinal(-20);
					} else if(after_transform_accel_linear.getX() >= 20) {
						coreData.getAccelSet().setLongitudinal(20);
					} else {
						coreData.getAccelSet().setLongitudinal((float) after_transform_accel_linear.getX());
					}
					
					if(after_transform_accel_linear.getY() <= -20) {
						coreData.getAccelSet().setLateral(-20);
					} else if(after_transform_accel_linear.getY() >= 20) {
						coreData.getAccelSet().setLateral(20);
					} else {
						coreData.getAccelSet().setLateral((float) after_transform_accel_linear.getY());
					}

					if(after_transform_accel_linear.getZ() <= -24.696) {
						coreData.getAccelSet().setLateral((float) -24.696);
					} else if(after_transform_accel_linear.getZ() >= 24.892) {
						coreData.getAccelSet().setLateral((float) 24.892);
					} else {
						coreData.getAccelSet().setLateral((float) after_transform_accel_linear.getZ());
					}
				}
				
				if(accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ() >= -327.67
						&& accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ() <= 327.67) {
					coreData.getAccelSet().setYawRate((float) accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ());
				}
			}
			
			coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 16);
			if(brake_ready) {
				if(brakeSubscriber.getLastMessage().getData() >= 0) {
					coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 15);
				} else {
					coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus((byte) 0);
				}
			}
			
			// N/A for now
			coreData.getBrakes().getTraction().setTractionControlStatus((byte) 0);
			coreData.getBrakes().getAbs().setAntiLockBrakeStatus((byte) 0);
			coreData.getBrakes().getScs().setStabilityControlStatus((byte) 0);
			coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied((byte) 0);
			coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus((byte) 0);

			// Set length and width only for the first time
			coreData.getSize().setVehicleLength(vehicleLength);
			coreData.getSize().setVehicleWidth(vehicleWidth);
			
			// Use ros node time
			coreData.setSecMark((short) ((node.getCurrentTime().toSeconds() * 1000) % DSECOND_MAX));

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
