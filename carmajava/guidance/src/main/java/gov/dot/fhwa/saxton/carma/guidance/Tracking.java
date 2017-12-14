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
import geometry_msgs.AccelStamped;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.Arbitrator;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
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
import org.ros.rosjava_geometry.Vector3;

import com.google.common.util.concurrent.AtomicDouble;

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

/**
 * Guidance package Tracking component
 * <p>
 * Responsible for detecting when the vehicle strays from its intended route or
 * trajectory and signaling the failure on the /system_alert topic. Also responsible
 * for generating content for BSMs to be published by the host vehicle.
 */
public class Tracking extends GuidanceComponent implements IStateChangeListener {
	
	protected final int SECONDS_TO_MILLISECONDS = 1000;
	protected final long SLEEP_DURATION = 100; // Frequency for J2735, 10Hz
	
	// TODO: brake information on each individual wheel is not available
	// When brake is applied at any angle, we set brake status to be 0xF
	protected final byte BRAKES_STATUS_UNAVAILABLE = 0x10;
	protected final byte BRAKES_NOT_APPLIED = 0x0;
	protected final byte BRAKES_APPLIED = 0xF;
	
	// Member variables
	protected long msgCount = 1;
	protected int last_id_changed = 0;
	protected float vehicleWidth = 0;
	protected float vehicleLength = 0;
	protected AtomicBoolean velocity_ready = new AtomicBoolean(false);;
	protected AtomicDouble current_speed = new AtomicDouble(0);
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
	protected ISubscriber<std_msgs.Bool> tractionActiveSubscriber;
	protected ISubscriber<std_msgs.Bool> tractionEnabledSubscriber;
	protected ISubscriber<std_msgs.Bool> antilockBrakeSubscriber;
	protected ISubscriber<std_msgs.Bool> stabilityActiveSubscriber;
	protected ISubscriber<std_msgs.Bool> stabilityEnabledSubscriber;
	protected ISubscriber<std_msgs.Bool> parkingBrakeSubscriber;
	protected IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> getDriversWithCapabilitiesClient;
	protected List<String> req_drivers = Arrays.asList(
			"steering_wheel_angle", "brake_position", "transmission_state",
			"traction_ctrl_active", "traction_ctrl_enabled", "antilock_brakes_active",
			"stability_ctrl_active", "stability_ctrl_enabled", "parking_brake");
	protected double speed_error_limit = 5; //speed error in meters
	protected double downtrack_error_limit = 5; //downtrack error in meters
	protected TrajectoryExecutor trajectoryExecutor = null;
	protected Arbitrator arbitrator = null;
	protected AtomicBoolean trajectory_start = new AtomicBoolean(false);
	protected Queue<Trajectory> trajectoryQueue = new LinkedList<Trajectory>();
	protected TreeMap<Long, double[]> speedTimeTree = new TreeMap<Long, double[]>((a, b) -> Long.compare(a, b));
	protected double trajectoryStartLocation = 0;
	protected long trajectoryStartTime = 0;
	

	public Tracking(GuidanceStateMachine stateMachine, IPubSubService pubSubService, ConnectedNode node) {
		super(stateMachine, pubSubService, node);
		jobQueue.add(this::onStartup);
		stateMachine.registerStateChangeListener(this);
	}

	@Override
	public String getComponentName() {
		return "Guidance.Tracking";
	}

    @Override
    public void onStartup() {
        // Publishers
        bsmPublisher = pubSubService.getPublisherForTopic("bsm", BSM._TYPE);

        // Subscribers
        navSatFixSubscriber = pubSubService.getSubscriberForTopic("nav_sat_fix", NavSatFix._TYPE);
        headingStampedSubscriber = pubSubService.getSubscriberForTopic("heading", HeadingStamped._TYPE);
        velocitySubscriber = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
        routeSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);

        // TODO: acceleration set is not available from SF
        accelerationSubscriber = pubSubService.getSubscriberForTopic("acceleration", AccelerationSet4Way._TYPE);

        if (bsmPublisher == null || navSatFixSubscriber == null || headingStampedSubscriber == null
                || velocitySubscriber == null || accelerationSubscriber == null || routeSubscriber == null) {
            log.warn("Cannot initialize pubs and subs");
        }

        velocitySubscriber.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override
            public void onMessage(TwistStamped msg) {
                current_speed.set(msg.getTwist().getLinear().getX());
                if (!velocity_ready.get()) {
                    velocity_ready.set(true);
                }
            }
        });
        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        // Make service call to get drivers
        log.debug("Trying to get get_drivers_with_capabilities service...");
        try {
            getDriversWithCapabilitiesClient = pubSubService.getServiceForTopic("get_drivers_with_capabilities",
                    GetDriversWithCapabilities._TYPE);
        } catch (TopicNotFoundException tnfe) {
            exceptionHandler.handleException("get_drivers_with_capabilities service cannot be found", tnfe);
        }
        if (getDriversWithCapabilitiesClient == null) {
            log.warn("get_drivers_with_capabilities service can not be found");
            exceptionHandler.handleException("get_drivers_with_capabilities service cannot be found",
                    new TopicNotFoundException());
        }

        GetDriversWithCapabilitiesRequest driver_request_wrapper = getDriversWithCapabilitiesClient.newMessage();
        driver_request_wrapper.setCapabilities(req_drivers);

        final GetDriversWithCapabilitiesResponse[] response = new GetDriversWithCapabilitiesResponse[1];

        getDriversWithCapabilitiesClient.callSync(driver_request_wrapper,
                new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {

                    @Override
                    public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
                        response[0] = msg;
                        log.debug("Tracking: service call is successful: " + response[0].getDriverData());
                    }

                    @Override
                    public void onFailure(Exception e) {
                        exceptionHandler.handleException("getDriversWithCapabilities service call failed", e);
                    }

                });

        if (response[0] == null || response[0].getDriverData().size() == 0) {
            exceptionHandler.handleException("cannot find suitable CAN driver",
                    new RosRuntimeException("No CAN driver is found"));
        }

        for (String driver_url : response[0].getDriverData()) {
            if (driver_url.endsWith("/can/steering_wheel_angle")) {
                steeringWheelSubscriber = pubSubService.getSubscriberForTopic(driver_url, Float64._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/brake_position")) {
                brakeSubscriber = pubSubService.getSubscriberForTopic(driver_url, Float64._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/transmission_state")) {
                transmissionSubscriber = pubSubService.getSubscriberForTopic(driver_url, TransmissionState._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/traction_ctrl_active")) {
                tractionActiveSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/traction_ctrl_enabled")) {
                tractionEnabledSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/antilock_brakes_active")) {
                antilockBrakeSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/stability_ctrl_active")) {
                stabilityActiveSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/stability_ctrl_enabled")) {
                stabilityEnabledSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
                continue;
            }
            if (driver_url.endsWith("/can/parking_brake")) {
                parkingBrakeSubscriber = pubSubService.getSubscriberForTopic(driver_url, std_msgs.Bool._TYPE);
            }
        }

        if (steeringWheelSubscriber == null || brakeSubscriber == null || transmissionSubscriber == null
                || tractionActiveSubscriber == null || tractionEnabledSubscriber == null
                || antilockBrakeSubscriber == null || stabilityActiveSubscriber == null
                || stabilityEnabledSubscriber == null || parkingBrakeSubscriber == null) {
            log.warn("Tracking: initialize subs failed");
        }

        try {
            ParameterTree param = node.getParameterTree();
            vehicleLength = (float) param.getDouble("vehicle_length");
            vehicleWidth = (float) param.getDouble("vehicle_width");
            speed_error_limit = param.getDouble("~max_speed_error");
            downtrack_error_limit = param.getDouble("~max_downtrack_error");
        } catch (ParameterNotFoundException e1) {
            exceptionHandler.handleException("Cannot found all parameters", e1);
        } catch (ParameterClassCastException e2) {
            exceptionHandler.handleException("Cannot cast on the parameter type", e2);
        }

        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onRouteActive() {
        trajectoryExecutor.registerOnTrajectoryProgressCallback(0.0, new OnTrajectoryProgressCallback() {

            @Override
            public void onProgress(double pct) {
                if (trajectoryQueue.size() == 0) {
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

        trajectoryExecutor.registerOnTrajectoryProgressCallback(0.95, new OnTrajectoryProgressCallback() {

            @Override
            public void onProgress(double pct) {
                trajectory_start.set(false);
                speedTimeTree.clear();
            }
        });

        currentState.set(GuidanceState.INACTIVE);
    }

    @Override
    public void onEngaged() {
        currentState.set(GuidanceState.ENGAGED);
    }

    @Override
    public void onCleanRestart() {
        currentState.set(GuidanceState.DRIVERS_READY);

        // Reset member variables
        trajectoryQueue.clear();
        speedTimeTree.clear();
        trajectoryStartLocation = 0;
        trajectoryStartTime = 0;
        trajectory_start.set(false);
    }
	
	@Override
	public void timingLoop() throws InterruptedException {
		
		long loop_start = System.currentTimeMillis();
		
		if(currentState.get() == GuidanceState.DRIVERS_READY) {
			
			//publish content for a new BSM
			bsmPublisher.publish(composeBSMData());
			
			//log the current vehicle forward speed
			if(velocity_ready.get()) {
	            log.info("Current vehicle speed is " + current_speed.get() + " m/s");
	            if(trajectory_start.get() && routeSubscriber != null && routeSubscriber.getLastMessage() != null) {
	    			long currentTime = System.currentTimeMillis();
	    			double currentDowntrack = routeSubscriber.getLastMessage().getDownTrack();
	    			if(hasTrajectoryError(currentTime, currentDowntrack, current_speed.get())) {
	    				arbitrator.notifyTrajectoryFailure();
	    			}
	    		}
			}
		}
		
		long loop_end = System.currentTimeMillis();
		
		Thread.sleep(Math.max(SLEEP_DURATION - (loop_end - loop_start), 0));
	}

	private void constructSpeedTimeTree(List<LongitudinalManeuver> maneuvers) {
		
		speedTimeTree.clear();
		long lastEntryTime = 0;
		for(int i = 0; i < maneuvers.size(); i++) {
			// Add the state of the start point of the first maneuver to the tree
			if(i == 0) {
				LongitudinalManeuver m = maneuvers.get(0);
				double[] speedAndDistance = new double[2];
				speedAndDistance[0] = m.getStartSpeed();
				speedAndDistance[1] = trajectoryStartLocation;
				speedTimeTree.put(trajectoryStartTime, speedAndDistance);
				lastEntryTime = trajectoryStartTime;
			}
			// Add the state of end points of maneuvers to the tree
			LongitudinalManeuver m = maneuvers.get(i);
			double[] speedAndDistance = new double[2];
			speedAndDistance[0] = m.getTargetSpeed();
			speedAndDistance[1] = m.getEndDistance();
			long durationTime = (long) (Math.abs(m.getStartDistance() - m.getEndDistance()) / (0.5 * (m.getStartSpeed() + m.getTargetSpeed())));
			long predictFinishTime = lastEntryTime + durationTime * SECONDS_TO_MILLISECONDS;
			speedTimeTree.put(predictFinishTime, speedAndDistance);
			lastEntryTime = predictFinishTime;
		}
	}
	
	private boolean hasTrajectoryError(long currentT, double currentD, double currentV) {
		Entry<Long, double[]> floorEntry = speedTimeTree.floorEntry(currentT);
		Entry<Long, double[]> ceilingEntry = speedTimeTree.ceilingEntry(currentT);
		if(floorEntry == null || ceilingEntry == null) {
			// This means that we are under the control of a complex maneuver and stop tracking errors.
			return false;
		}
		// Calculate current progress percentage in the current maneuver
		double factor = 1;
		if(ceilingEntry.getKey() - floorEntry.getKey() != 0) {
			factor = (currentT - floorEntry.getKey()) / (ceilingEntry.getKey() - floorEntry.getKey());
		}
		// Validate current speed with target speed
		double speedChange = ceilingEntry.getValue()[0] - floorEntry.getValue()[0]; 
		double targetSpeed = floorEntry.getValue()[0] + factor * speedChange;
		double speed_error = currentV - targetSpeed;
		log.debug("Current speed error is " + speed_error);
		if(Math.abs(speed_error) > speed_error_limit) {
			log.warn("Speed error " + speed_error + " is larger than its limit!");
			return true;
		}
		// Validate downtrack distance
		double distanceChange = ceilingEntry.getValue()[1] - floorEntry.getValue()[1];
		double targetDistance = floorEntry.getValue()[1] + factor * distanceChange;
		double distance_error = currentD - targetDistance;
		log.debug("Current downtrack error is " + distance_error);
		if(Math.abs(distance_error) > downtrack_error_limit) {
			log.warn("Distance error: " + distance_error + " is larger than its limit!");
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
		if(navSatFixSubscriber != null && navSatFixSubscriber.getLastMessage() != null) {
			NavSatFix gps_msg = navSatFixSubscriber.getLastMessage();
			double lat = gps_msg.getLatitude();
			double Lon = gps_msg.getLongitude();
			float elev = (float) gps_msg.getAltitude();
			//TODO: need to have transformation from baselink frame to vehicle frame
			if(lat >= BSMCoreData.LATITUDE_MIN && lat <= BSMCoreData.LATITUDE_MAX) {
				coreData.setLatitude(lat);
			}
			if(Lon >= BSMCoreData.LONGITUDE_MIN && Lon <= BSMCoreData.LONGITUDE_MAX) {
				coreData.setLongitude(Lon);
			}
			if(elev >= BSMCoreData.ELEVATION_MIN && elev <= BSMCoreData.ELEVATION_MAX) {
				coreData.setElev(elev);
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
		if(transmissionSubscriber != null && transmissionSubscriber.getLastMessage() != null) {
			byte transmission_state = transmissionSubscriber.getLastMessage().getTransmissionState();
			if(transmission_state == TransmissionState.NEUTRAL
					|| transmission_state == TransmissionState.FORWARDGEARS
					|| transmission_state == TransmissionState.PARK
					|| transmission_state == TransmissionState.REVERSEGEARS) {
				coreData.getTransmission().setTransmissionState(transmission_state);
			}
		}

		coreData.setSpeed(BSMCoreData.SPEED_UNAVAILABLE);
		if(velocity_ready.get()) {
			float speed = (float) current_speed.get();
			if(speed >= BSMCoreData.SPEED_MIN && speed <= BSMCoreData.SPEED_MAX) {
				coreData.setSpeed(speed);
			}
		}
		
		coreData.setHeading(BSMCoreData.HEADING_UNAVAILABLE);
		if(headingStampedSubscriber != null && headingStampedSubscriber.getLastMessage() != null) {
			float heading = (float) headingStampedSubscriber.getLastMessage().getHeading();
			if(heading >= BSMCoreData.HEADING_MIN && heading <= BSMCoreData.HEADING_MAX) {
				coreData.setHeading(heading);
			}
		}
		
		coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_UNAVAILABLE);
		if(steeringWheelSubscriber != null && steeringWheelSubscriber.getLastMessage() != null) {
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
		if(accelerationSubscriber != null && accelerationSubscriber.getLastMessage() != null) {
			Vector3 accel_set_linear = Vector3.fromVector3Message(accelerationSubscriber.getLastMessage().getAccel().getLinear());
			AccelerationSet4Way acceleration = coreData.getAccelSet();
			if(accel_set_linear.getX() <= AccelerationSet4Way.ACCELERATION_MIN) {
				acceleration.setLongitudinal(AccelerationSet4Way.ACCELERATION_MIN);
			} else if(accel_set_linear.getX() >= AccelerationSet4Way.ACCELERATION_MAX) {
				acceleration.setLongitudinal(AccelerationSet4Way.ACCELERATION_MAX);
			} else {
				acceleration.setLongitudinal((float) accel_set_linear.getX());
			}
			
			if(accel_set_linear.getY() <= AccelerationSet4Way.ACCELERATION_MIN) {
				acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MIN);
			} else if(accel_set_linear.getY() >= AccelerationSet4Way.ACCELERATION_MAX) {
				acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MAX);
			} else {
				acceleration.setLateral((float) accel_set_linear.getY());
			}

			if(accel_set_linear.getZ() <= AccelerationSet4Way.ACCELERATION_VERTICAL_MIN) {
				acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MIN);
			} else if(accel_set_linear.getZ() >= AccelerationSet4Way.ACCELERATION_VERTICAL_MAX) {
				acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MAX);
			} else {
				acceleration.setVert((float) accel_set_linear.getZ());
			}
			
			double yaw_rate = accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ();
			if(yaw_rate >= AccelerationSet4Way.YAWRATE_MIN && yaw_rate <= AccelerationSet4Way.YAWRATE_MAX) {
				coreData.getAccelSet().setYawRate((float) yaw_rate);
			}
		}
		
		coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_STATUS_UNAVAILABLE);
		if(brakeSubscriber != null && brakeSubscriber.getLastMessage() != null) {
			if(brakeSubscriber.getLastMessage().getData() > BRAKES_NOT_APPLIED) {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_APPLIED);
			} else {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_NOT_APPLIED);
			}
		}
		
		
		coreData.getBrakes().getTraction().setTractionControlStatus(TractionControlStatus.UNAVAILABLE);
		coreData.getBrakes().getAbs().setAntiLockBrakeStatus(AntiLockBrakeStatus.UNAVAILABLE);
		coreData.getBrakes().getScs().setStabilityControlStatus(StabilityControlStatus.UNAVAILABLE);
		coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus(AuxiliaryBrakeStatus.UNAVAILABLE);
		// TODO: N/A for now
		coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(BrakeBoostApplied.UNAVAILABLE);
		
		if(tractionActiveSubscriber != null && tractionActiveSubscriber != null && tractionActiveSubscriber.getLastMessage() != null && tractionActiveSubscriber.getLastMessage().getData()) {
			if(tractionEnabledSubscriber != null && tractionEnabledSubscriber != null && tractionEnabledSubscriber.getLastMessage() != null && tractionEnabledSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getTraction().setTractionControlStatus(TractionControlStatus.ENGAGED);
			} else {
				coreData.getBrakes().getTraction().setTractionControlStatus(TractionControlStatus.ON);
			}
		}
		if(antilockBrakeSubscriber != null && antilockBrakeSubscriber != null && antilockBrakeSubscriber.getLastMessage() != null && antilockBrakeSubscriber.getLastMessage().getData()) {
			coreData.getBrakes().getAbs().setAntiLockBrakeStatus(AntiLockBrakeStatus.ON);
		}
		if(stabilityActiveSubscriber != null && stabilityActiveSubscriber.getLastMessage() != null && stabilityActiveSubscriber.getLastMessage().getData()) {
			if(stabilityEnabledSubscriber != null && stabilityEnabledSubscriber.getLastMessage() != null && stabilityEnabledSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getScs().setStabilityControlStatus(StabilityControlStatus.ENGAGED);
			} else {
				coreData.getBrakes().getScs().setStabilityControlStatus(StabilityControlStatus.ON);
			}
		}
		if(parkingBrakeSubscriber != null && parkingBrakeSubscriber.getLastMessage() != null) {
			if(parkingBrakeSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(BrakeBoostApplied.ON);
			} else {
				coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(BrakeBoostApplied.OFF);
			}
		}
		
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

    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onRouteActive);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }
}
