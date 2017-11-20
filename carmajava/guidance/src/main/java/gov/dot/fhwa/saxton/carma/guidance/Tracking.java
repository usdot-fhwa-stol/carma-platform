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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;

import gov.dot.fhwa.saxton.carma.guidance.trajectory.OnTrajectoryProgressCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import sensor_msgs.NavSatFix;
import std_msgs.Float64;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Random;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package Tracking component
 * <p>
 * Responsible for detecting when the vehicle strays from its intended route or
 * trajectory and signaling the failure on the /system_alert topic. Also responsible
 * for generating content for BSMs to be published by the host vehicle.
 */
public class Tracking extends GuidanceComponent {
	
	protected static final int SECONDS_TO_MILLISECONDS = 1000;
	
	// TODO: brake information on each individual wheel is not available
	// When brake is applied at any angle, we set brake status to be 0xF
	protected static final byte BRAKES_STATUS_UNAVAILABLE = 0x10;
	protected static final byte BRAKES_NOT_APPLIED = 0x0;
	protected static final byte BRAKES_APPLIED = 0xF;

	// Member variables
	protected final long sleepDurationMillis = 100; // Frequency for J2735, 10Hz
	protected long msgCount = 1;
	protected int last_id_changed = 0;
	protected float vehicleWidth = 0;
	protected float vehicleLength = 0;
	protected AtomicBoolean drivers_ready = new AtomicBoolean(false);
	protected boolean steer_wheel_ready = false;
	protected boolean nav_sat_fix_ready = false;
	protected boolean heading_ready = false;
	protected boolean velocity_ready = false;
	protected boolean brake_ready = false;
	protected boolean transmission_ready = false;
	protected boolean acceleration_ready = false;
	protected boolean vehicle_to_baselink_transform_ready = false;
	protected boolean base_to_map_transform_ready = false;
	protected boolean route_state_ready = false;
	protected GuidanceExceptionHandler exceptionHandler;
	protected Random randomIdGenerator = new Random();
	protected byte[] random_id = new byte[4];
	protected IPublisher<BSM> bsmPublisher;
	protected ISubscriber<AccelStamped> accelerationSubscriber;
	protected ISubscriber<NavSatFix> navSatFixSubscriber;
	protected ISubscriber<HeadingStamped> headingStampedSubscriber;
	protected ISubscriber<TwistStamped> velocitySubscriber;
	protected ISubscriber<Float64> steeringWheelSubscriber;
	protected ISubscriber<Float64> brakeSubscriber;
	protected ISubscriber<TransmissionState> transmissionSubscriber;
	protected ISubscriber<RouteState> routeSubscriber;
	protected IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient;
	protected IService<GetTransformRequest, GetTransformResponse> getTransformClient;
	protected List<String> req_drivers = Arrays.asList("steering_wheel_angle", "brake_position", "transmission_state");
	protected List<String> resp_drivers;
	protected final String baseLinkFrame = "base_link";
	protected final String vehicleFrame = "host_vehicle";
	protected final String mapFrame = "map";
	protected final String earthFrame = "earth";
	protected Transform vehicleToBaselink = null;
	protected Transform baseToMap = null;
	protected GeodesicCartesianConverter converter = new GeodesicCartesianConverter();
	
	protected double speed_error_limit = 0; //speed error in meters
	protected double downtrack_error_limit = 0; //downtrack error in meters
	protected TrajectoryExecutor trajectoryExecutor = null;
	protected Arbitrator arbitrator = null;
	protected AtomicBoolean trajectory_start = new AtomicBoolean(false);
	protected Queue<Trajectory> trajectoryQueue = new LinkedList<Trajectory>();
	protected TreeMap<Long, double[]> speedTimeTree = new TreeMap<Long, double[]>((a, b) -> (a - b < 0 ? -1 : 1));
	protected double trajectoryStartLocation = 0;
	protected long trajectoryStartTime = 0;
	

	public Tracking(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
		super(state, pubSubService, node);
		this.exceptionHandler = new GuidanceExceptionHandler(state);
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
		headingStampedSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);
		velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
		routeSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
		// TODO: acceleration set is not available from SF
		accelerationSubscriber = pubSubService.getSubscriberForTopic("acceleration", AccelerationSet4Way._TYPE);
		
		if(bsmPublisher == null 
				|| navSatFixSubscriber == null 
				|| headingStampedSubscriber == null 
				|| velocitySubscriber == null 
				|| accelerationSubscriber == null) {
			log.warn("Cannot initialize pubs and subs");
		}
		
		navSatFixSubscriber.registerOnMessageCallback(new OnMessageCallback<NavSatFix>() {
			@Override
			public void onMessage(NavSatFix msg) {
				if(!nav_sat_fix_ready) {
					nav_sat_fix_ready = true;
					log.info("BSM", "nav_sat_fix subscriber is ready");
				}
			}
		});
		
		
		headingStampedSubscriber.registerOnMessageCallback(new OnMessageCallback<HeadingStamped>() {
			@Override
			public void onMessage(HeadingStamped msg) {
				if(!heading_ready) {
					heading_ready = true;
					log.info("BSM", "heading subscriber is ready");
				}
			}
		});
		
		velocitySubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
			@Override
			public void onMessage(TwistStamped msg) {
				if(!velocity_ready) {
					velocity_ready = true;
					log.info("BSM", "velocity subscriber is ready");
				}
			}
		});
		
		accelerationSubscriber.registerOnMessageCallback(new OnMessageCallback<AccelStamped>() {
			@Override
			public void onMessage(AccelStamped msg) {
				if(!acceleration_ready) {
					acceleration_ready = true;
					log.info("BSM", "acceleration subscriber is ready");
				}
			}
		});
		
		routeSubscriber.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
			@Override
			public void onMessage(RouteState msg) {
				if(!route_state_ready) {
					route_state_ready = true;
					log.info("route state subscriber is ready");
				}
			}
		});
	}

	@Override
	public void onSystemReady() {
		
		// Make service call to get drivers
		log.info("Trying to get get_drivers_with_capabilities service...");
		try {
			getDriversWithCapabilitiesClient = pubSubService.getServiceForTopic("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
		} catch (TopicNotFoundException e1) {
			handleException(e1);
		}
		if(getDriversWithCapabilitiesClient == null) {
			log.warn("get_drivers_with_capabilities service can not be found");
		}
		
		GetDriversWithCapabilitiesRequest driver_request_wrapper = getDriversWithCapabilitiesClient.newMessage();
		driver_request_wrapper.setCapabilities(req_drivers);
		getDriversWithCapabilitiesClient.callSync(driver_request_wrapper, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
			
			@Override
			public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
				resp_drivers = msg.getDriverData();
				log.info("Tracking: service call is successful: " + resp_drivers);
			}
			
			@Override
			public void onFailure(Exception e) {
				throw new RosRuntimeException(e);
			}
			
		});
		
		log.info("Trying to get get_transform...");
		try {
			getTransformClient = pubSubService.getServiceForTopic("get_transform", GetTransform._TYPE);
		} catch (TopicNotFoundException e1) {
			handleException(e1);
		}
		if(getTransformClient == null) {
			log.warn("get_transform service can not be found");
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
			log.warn("Tracking: cannot find suitable drivers");
		}
		
		if(steeringWheelSubscriber == null || brakeSubscriber == null || transmissionSubscriber == null) {
			log.warn("Tracking: initialize subs failed");
		}
		
		steeringWheelSubscriber.registerOnMessageCallback(new OnMessageCallback<Float64>() {
			@Override
			public void onMessage(Float64 msg) {
				if(!steer_wheel_ready) {
					steer_wheel_ready = true;
					log.info("BSM", "steer_wheel subscriber is ready");
				}
			}
		});
		
		brakeSubscriber.registerOnMessageCallback(new OnMessageCallback<Float64>() {
			@Override
			public void onMessage(Float64 msg) {
				if(!brake_ready) {
					brake_ready = true;
					log.info("BSM", "brake subscriber is ready");
				}
			}
		});
		
		transmissionSubscriber.registerOnMessageCallback(new OnMessageCallback<TransmissionState>() {
			@Override
			public void onMessage(TransmissionState msg) {
				if(!transmission_ready) {
					transmission_ready = true;
					log.info("BSM", "transmission subscriber is ready");
				}
			}
		});
		
		try {
			ParameterTree param = node.getParameterTree();
			vehicleLength = (float) param.getDouble("vehicle_length");
			vehicleWidth = (float) param.getDouble("vehicle_width");
			speed_error_limit = param.getDouble("~max_speed_error");
			downtrack_error_limit = param.getDouble("~max_downtrack_error");
		} catch (ParameterNotFoundException e1) {
			handleException(e1);
		} catch (ParameterClassCastException e2) {
			handleException(e2);
		}
		
		drivers_ready.set(true);
	}

	@Override
	public void onGuidanceEnable() {
		this.trajectoryExecutor.registerOnTrajectoryProgressCallback(0.0, new OnTrajectoryProgressCallback() {
			
			@Override
			public void onProgress(double pct) {
				if(trajectoryQueue.size() == 0) {
					log.warn("Cannot track errors for current Trajectory!");
				} else {
					Trajectory currentTrajectory = trajectoryQueue.poll();
					trajectoryStartLocation = currentTrajectory.getStartLocation(); 
					trajectoryStartTime = System.currentTimeMillis();
					constructSpeedTimeTree(currentTrajectory.getLongitudinalManeuvers());
					trajectory_start.set(true);
				}
			}
		});
		this.trajectoryExecutor.registerOnTrajectoryProgressCallback(1.0, new OnTrajectoryProgressCallback() {
			
			@Override
			public void onProgress(double pct) {
				trajectory_start.set(false);
				speedTimeTree.clear();
			}
		});
	}

	@Override
	public void loop() throws InterruptedException {
		
		if(drivers_ready.get()) {
			//publish content for a new BSM
			bsmPublisher.publish(composeBSMData());
			//log the current vehicle forward speed
            TwistStamped vel = velocitySubscriber.getLastMessage();
            if(vel != null) {
                double speed = vel.getTwist().getLinear().getX();
                log.info("Current vehicle speed is " + speed + " m/s");
            }
		}
		if(trajectory_start.get() && routeSubscriber.getLastMessage() != null && velocitySubscriber != null) {
			long currentTime = System.currentTimeMillis();
			double currentDowntrack = routeSubscriber.getLastMessage().getDownTrack();
			double currentSpeed = velocitySubscriber.getLastMessage().getTwist().getLinear().getX();
			if(hasTrajectoryError(currentTime, currentDowntrack, currentSpeed)) {
				arbitrator.needsReplan.set(true);
			}
		}
		Thread.sleep(sleepDurationMillis);
	}

	private void constructSpeedTimeTree(List<IManeuver> maneuvers) {
		
		speedTimeTree.clear();
		long lastEntryTime = 0;
		for(int i = 0; i < maneuvers.size(); i++) {
			// Add the state of the start point of the first maneuver to the tree
			if(i == 0) {
				IManeuver m = maneuvers.get(0);
				double[] speedAndDistance = new double[2];
				speedAndDistance[0] = m.getStartSpeed();
				speedAndDistance[1] = trajectoryStartLocation;
				speedTimeTree.put(trajectoryStartTime, speedAndDistance);
				lastEntryTime = trajectoryStartTime;
			}
			// Add the state of end points of maneuvers to the tree
			IManeuver m = maneuvers.get(i);
			double[] speedAndDistance = new double[2];
			speedAndDistance[0] = m.getTargetSpeed();
			speedAndDistance[1] = m.getEndDistance();
			long finishTime = (long) (Math.abs(m.getStartDistance() - m.getEndDistance()) / 0.5 * (m.getStartSpeed() + m.getTargetSpeed()));
			long predictFinishTime = lastEntryTime + finishTime;
			speedTimeTree.put(predictFinishTime, speedAndDistance);
			lastEntryTime = predictFinishTime;
		}
	}
	
	private boolean hasTrajectoryError(long currentT, double currentD, double currentV) {
		Entry<Long, double[]> floorEntry = speedTimeTree.floorEntry(currentT);
		Entry<Long, double[]> ceilingEntry = speedTimeTree.ceilingEntry(currentT);
		if(floorEntry == null || ceilingEntry == null) {
			// This means that we are under the control of a complex maneuver and stop tracking errors. 
			return true;
		}
		// Calculate current progress percentage in the current maneuver
		double factor = (currentT - floorEntry.getKey()) / (ceilingEntry.getKey() - floorEntry.getKey());
		// Validate current speed with target speed
		double speedChange = ceilingEntry.getValue()[0] - floorEntry.getValue()[0]; 
		double targetSpeed = floorEntry.getValue()[0] + factor * speedChange;
		if(Math.abs(targetSpeed - currentV) > speed_error_limit) {
			return true;
		}
		// Validate downtrack distance
		double distanceChange = ceilingEntry.getValue()[1] - floorEntry.getValue()[1];
		double targetDistance = floorEntry.getValue()[1] + factor * distanceChange;
		if(Math.abs(targetDistance - currentD) > downtrack_error_limit) {
			return true;
		}
		return false;
	}

	private BSM composeBSMData() {

		BSM bsmFrame = bsmPublisher.newMessage();
		
		// Set header
		bsmFrame.getHeader().setStamp(node.getCurrentTime());
		bsmFrame.getHeader().setFrameId(Tracking.class.getSimpleName());

		// Set core data
		BSMCoreData coreData = bsmFrame.getCoreData();
		coreData.setMsgCount((byte) ((msgCount++ % BSMCoreData.MSG_COUNT_MAX) + 1));

		// ID is random and changes every 5 minutes
		if(node.getCurrentTime().secs - last_id_changed >= BSMCoreData.ID_TIME_MAX) {
			randomIdGenerator.nextBytes(random_id);
			last_id_changed = node.getCurrentTime().secs;
		}
		coreData.setId(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, random_id));

		// Set GPS data
		coreData.setLatitude(BSMCoreData.LATITUDE_UNAVAILABLE); // Default value when unknown
		coreData.setLongitude(BSMCoreData.LONGITUDE_UNAVAILABLE);
		coreData.setElev(BSMCoreData.ELEVATION_UNAVAILABLE);
		coreData.getAccuracy().setSemiMajor(PositionalAccuracy.ACCURACY_UNAVAILABLE);
		coreData.getAccuracy().setSemiMinor(PositionalAccuracy.ACCURACY_UNAVAILABLE);
		coreData.getAccuracy().setOrientation(PositionalAccuracy.ACCURACY_ORIENTATION_UNAVAILABLE);
		if(navSatFixSubscriber.getLastMessage() != null && getTransformClient != null) {
			NavSatFix gps_msg = navSatFixSubscriber.getLastMessage();
			Location location_on_baselink = new Location();
			location_on_baselink.setLongitude(gps_msg.getLongitude());
			location_on_baselink.setLatitude(gps_msg.getLatitude());
			location_on_baselink.setAltitude(gps_msg.getAltitude());
			Point3D point_on_baselink = converter.geodesic2Cartesian(location_on_baselink, Transform.identity());
			if(!vehicle_to_baselink_transform_ready) {
				GetTransformRequest transform_request = getTransformClient.newMessage();
				transform_request.setParentFrame(vehicleFrame);
				transform_request.setChildFrame(baseLinkFrame);
				getTransformClient.callSync(transform_request, new OnServiceResponseCallback<GetTransformResponse>() {
					
					@Override
					public void onSuccess(GetTransformResponse msg) {
						log.debug("BSM", "Get baselink_to_vehicle_transform response: " + (msg.getErrorStatus() == 0 ? "Successed" : "Failed"));
						if(msg.getErrorStatus() == 0) {
							vehicleToBaselink = Transform.fromTransformMessage(msg.getTransform().getTransform());
							vehicle_to_baselink_transform_ready = true;
						}
					}
					
					@Override
					public void onFailure(Exception e) {
					}
				});
			}
			
			if(vehicle_to_baselink_transform_ready) {
				Vector3 after_transform = vehicleToBaselink.apply(new Vector3(point_on_baselink.getX(), point_on_baselink.getY(), point_on_baselink.getZ()));
				Point3D point_on_vehicle = new Point3D(after_transform.getX(), after_transform.getY(), after_transform.getZ());
				Location location_on_vehicle = converter.cartesian2Geodesic(point_on_vehicle, Transform.identity());
				double lat = location_on_vehicle.getLatitude();
				double Lon = location_on_vehicle.getLongitude();
				float elev = (float) location_on_vehicle.getAltitude();
				if(lat >= BSMCoreData.LATITUDE_MIN && lat <= BSMCoreData.LATITUDE_MAX) {
					coreData.setLatitude(lat);
				}
				if(Lon >= BSMCoreData.LONGITUDE_MIN && Lon <= BSMCoreData.LONGITUDE_MAX) {
					coreData.setLongitude(Lon);
				}
				if(elev >= BSMCoreData.ELEVATION_MIN && elev <= BSMCoreData.ELEVATION_MAX) {
					coreData.setElev(elev);
				}
			}
			
			double semi_major_square = gps_msg.getPositionCovariance()[0];
			double semi_minor_square = gps_msg.getPositionCovariance()[4];
			// Orientation of semi_major axis
			// Orientation of accuracy eclipse is fixed to north based on Pinpoint documentations
			// TODO: May need to change if we use other Pinpoints
			double orientation = PositionalAccuracy.ACCURACY_ORIENTATION_MIN; 
			
			float semi_major = -1;
			float semi_minor = -1;
			if(semi_major_square >= 0) {
				semi_major = (float) Math.sqrt(semi_major_square);
			}
			if(semi_minor_square >= 0) {
				semi_minor = (float) Math.sqrt(semi_minor_square);
			}
			if(semi_major != -1) {
				if(semi_major >= PositionalAccuracy.ACCURACY_MAX) {
					coreData.getAccuracy().setSemiMajor(PositionalAccuracy.ACCURACY_MAX);
				} else {
					coreData.getAccuracy().setSemiMajor(semi_major);
				}
			}
			
			if(semi_minor != -1) {
				if(semi_minor >= PositionalAccuracy.ACCURACY_MAX) {
					coreData.getAccuracy().setSemiMinor(PositionalAccuracy.ACCURACY_MAX);
				} else {
					coreData.getAccuracy().setSemiMinor(semi_minor);
				}
			}
			
			if(orientation >= PositionalAccuracy.ACCURACY_ORIENTATION_MIN && orientation <= PositionalAccuracy.ACCURACY_ORIENTATION_MAX) {
				coreData.getAccuracy().setOrientation(orientation);
			}
		}
		
		// Set transmission state
		// Reserved values are illegal values at this time
		coreData.getTransmission().setTransmissionState(TransmissionState.UNAVAILABLE);
		if(transmissionSubscriber.getLastMessage() != null) {
			byte transmission_state = transmissionSubscriber.getLastMessage().getTransmissionState();
			if(transmission_state == TransmissionState.NEUTRAL
					|| transmission_state == TransmissionState.FORWARDGEARS
					|| transmission_state == TransmissionState.PARK
					|| transmission_state == TransmissionState.REVERSEGEARS) {
				coreData.getTransmission().setTransmissionState(transmission_state);
			}
		}

		coreData.setSpeed(BSMCoreData.SPEED_UNAVAILABLE);
		if(velocitySubscriber.getLastMessage() != null) {
			float speed = (float) velocitySubscriber.getLastMessage().getTwist().getLinear().getX();
			if(speed >= BSMCoreData.SPEED_MIN && speed <= BSMCoreData.SPEED_MAX) {
				coreData.setSpeed(speed);
			}
		}
		
		coreData.setHeading(BSMCoreData.HEADING_UNAVAILABLE);
		if(headingStampedSubscriber.getLastMessage() != null) {
			float heading = (float) headingStampedSubscriber.getLastMessage().getHeading();
			if(heading >= BSMCoreData.HEADING_MIN && heading <= BSMCoreData.HEADING_MAX) {
				coreData.setHeading(heading);
			}
		}
		
		coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_UNAVAILABLE);
		if(steeringWheelSubscriber.getLastMessage() != null) {
			float angle = (float) steeringWheelSubscriber.getLastMessage().getData();	
			if(angle <= BSMCoreData.STEER_WHEEL_ANGLE_MIN) {
				coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_MIN);
			} else if(angle >= BSMCoreData.STEER_WHEEL_ANGLE_MAX) {
				coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_MAX);
			} else {
				coreData.setAngle(angle);
			}
		}

		coreData.getAccelSet().setLongitudinal(AccelerationSet4Way.ACCELERATION_UNAVAILABLE);
		coreData.getAccelSet().setLateral(AccelerationSet4Way.ACCELERATION_UNAVAILABLE);
		coreData.getAccelSet().setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_UNAVAILABLE);
		// TODO: It is not well defined in J2735
		coreData.getAccelSet().setYawRate(AccelerationSet4Way.YAWRATE_UNAVAILABLE);
		if(accelerationSubscriber.getLastMessage() != null && getTransformClient != null) {
			GetTransformRequest transform_request = getTransformClient.newMessage();
			transform_request.setParentFrame(mapFrame);
			transform_request.setChildFrame(baseLinkFrame);
			getTransformClient.callSync(transform_request, new OnServiceResponseCallback<GetTransformResponse>() {
				
				@Override
				public void onSuccess(GetTransformResponse msg) {
					log.debug("BSM", "Get base_to_map_transform response " + (msg.getErrorStatus() == 0 ? "Successed" : "Failed"));
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
				Vector3 after_transform_accel_linear = baseToMap.apply(Vector3.fromVector3Message(accelerationSubscriber.getLastMessage().getAccel().getLinear()));
				AccelerationSet4Way acceleration = coreData.getAccelSet();
				if(after_transform_accel_linear.getX() <= AccelerationSet4Way.ACCELERATION_MIN) {
					acceleration.setLongitudinal(AccelerationSet4Way.ACCELERATION_MIN);
				} else if(after_transform_accel_linear.getX() >= AccelerationSet4Way.ACCELERATION_MAX) {
					acceleration.setLongitudinal(AccelerationSet4Way.ACCELERATION_MAX);
				} else {
					acceleration.setLongitudinal((float) after_transform_accel_linear.getX());
				}
				
				if(after_transform_accel_linear.getY() <= AccelerationSet4Way.ACCELERATION_MIN) {
					acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MIN);
				} else if(after_transform_accel_linear.getY() >= AccelerationSet4Way.ACCELERATION_MAX) {
					acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MAX);
				} else {
					acceleration.setLateral((float) after_transform_accel_linear.getY());
				}

				if(after_transform_accel_linear.getZ() <= AccelerationSet4Way.ACCELERATION_VERTICAL_MIN) {
					acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MIN);
				} else if(after_transform_accel_linear.getZ() >= AccelerationSet4Way.ACCELERATION_VERTICAL_MAX) {
					acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MAX);
				} else {
					acceleration.setVert((float) after_transform_accel_linear.getZ());
				}
			}
			
			if(accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ() >= AccelerationSet4Way.YAWRATE_MIN
					&& accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ() <= AccelerationSet4Way.YAWRATE_MAX) {
				coreData.getAccelSet().setYawRate((float) accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ());
			}
		}
		
		coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_STATUS_UNAVAILABLE);
		if(brakeSubscriber.getLastMessage() != null) {
			if(brakeSubscriber.getLastMessage().getData() > BRAKES_NOT_APPLIED) {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_APPLIED);
			} else {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_NOT_APPLIED);
			}
		}
		
		// TODO: N/A for now
		coreData.getBrakes().getTraction().setTractionControlStatus(TractionControlStatus.UNAVAILABLE);
		coreData.getBrakes().getAbs().setAntiLockBrakeStatus(AntiLockBrakeStatus.UNAVAILABLE);
		coreData.getBrakes().getScs().setStabilityControlStatus(StabilityControlStatus.UNAVAILABLE);
		coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(BrakeBoostApplied.UNAVAILABLE);
		coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus(AuxiliaryBrakeStatus.UNAVAILABLE);

		// Set length and width only for the first time
		coreData.getSize().setVehicleLength(VehicleSize.VEHICLE_LENGTH_UNAVAILABLE);
		coreData.getSize().setVehicleWidth(VehicleSize.VEHICLE_WIDTH_UNAVAILABLE);
		if(vehicleLength >= VehicleSize.VEHICLE_LENGTH_MIN && vehicleLength <= VehicleSize.VEHICLE_LENGTH_MAX) {
			coreData.getSize().setVehicleLength(vehicleLength);
		}
		if(vehicleWidth >= VehicleSize.VEHICLE_WIDTH_MIN && vehicleWidth <= VehicleSize.VEHICLE_WIDTH_MAX) {
			coreData.getSize().setVehicleWidth(vehicleWidth);
		}
		
		// Use ros node time and ignore leap second for now since it is not announced
		coreData.setSecMark((int) (System.currentTimeMillis() % BSMCoreData.SEC_MARK_MOD));

		return bsmFrame;
	}

	public void addNewTrajectory(Trajectory traj) {
		trajectoryQueue.add(traj);
	}
	
	public void setTrajectoryExecutor(TrajectoryExecutor trajectoryExecutor) {
		this.trajectoryExecutor = trajectoryExecutor;
	}
	
	public void setArbitrator(Arbitrator arbitrator) {
		this.arbitrator = arbitrator;
	}
	
	protected void handleException(Exception e) {
		log.error("Tracking throws an exception...");
		exceptionHandler.handleException(e);
	}
}
