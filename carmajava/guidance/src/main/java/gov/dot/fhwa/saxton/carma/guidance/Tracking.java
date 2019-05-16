/*
 * Copyright (C) 2018-2019 LEIDOS.
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
import cav_srvs.*;
import geometry_msgs.AccelStamped;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
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
import gov.dot.fhwa.saxton.carma.guidance.trajectory.TrajectoryExecutor;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.exception.RosRuntimeException;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import com.google.common.util.concurrent.AtomicDouble;

import sensor_msgs.NavSatFix;
import std_msgs.Float64;

import java.nio.ByteOrder;
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
public class Tracking extends GuidanceComponent implements IStateChangeListener, TrackingService {
	
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
	protected AtomicBoolean velocity_ready = new AtomicBoolean(false);
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
	protected ISubscriber<j2735_msgs.TransmissionState> transmissionSubscriber;
	protected ISubscriber<RouteState> routeSubscriber;
	protected ISubscriber<std_msgs.Bool> tractionActiveSubscriber;
	protected ISubscriber<std_msgs.Bool> tractionEnabledSubscriber;
	protected ISubscriber<std_msgs.Bool> antilockBrakeSubscriber;
	protected ISubscriber<std_msgs.Bool> stabilityActiveSubscriber;
	protected ISubscriber<std_msgs.Bool> stabilityEnabledSubscriber;
	protected ISubscriber<std_msgs.Bool> parkingBrakeSubscriber;
	protected IService<GetTransformRequest, GetTransformResponse> getTransformClient;
	protected double speed_error_limit = 5; //speed error in meters
	protected double downtrack_error_limit = 5; //downtrack error in meters
	protected TrajectoryExecutor trajectoryExecutor = null;
	protected Arbitrator arbitrator = null;
	protected AtomicBoolean trajectory_start = new AtomicBoolean(false);
	protected Queue<Trajectory> trajectoryQueue = new LinkedList<Trajectory>();
	protected TreeMap<Long, double[]> speedTimeTree = new TreeMap<Long, double[]>((a, b) -> Long.compare(a, b));
	protected double trajectoryStartLocation = 0;
	protected long trajectoryStartTime = 0;
	
	private static final String CAN_DRIVER_BASE_PATH = "/can/";
	private static final String STEERING_WHEEL_CAPABILITY = "steering_wheel_angle";
	private static final String BRAKE_POSITION_CAPABILITY = "brake_position";
	private static final String TRANSMISSION_STATE_CAPABILITY = "transmission_state";
	private static final String TRACTION_CTRL_ACTIVE_CAPABILITY = "traction_ctrl_active";
	private static final String TRACTION_CTRL_ENABLED_CAPABILITY = "traction_ctrl_enabled";
	private static final String ANTILOCK_BRAKES_ACTIVE_CAPABILITY = "antilock_brakes_active";
	private static final String STABILITY_CTRL_ACTIVE_CAPABILITY = "stability_ctrl_active";
	private static final String STABILITY_CTRL_ENABLED_CAPABILITY = "stability_ctrl_enabled";
	private static final String PARKING_BRAKE_CAPABILITY = "parking_brake";
	

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

        steeringWheelSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + STEERING_WHEEL_CAPABILITY, Float64._TYPE);
        brakeSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + BRAKE_POSITION_CAPABILITY, Float64._TYPE);
        transmissionSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + TRANSMISSION_STATE_CAPABILITY, j2735_msgs.TransmissionState._TYPE);
        tractionActiveSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + TRACTION_CTRL_ACTIVE_CAPABILITY, std_msgs.Bool._TYPE);
        tractionEnabledSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + TRACTION_CTRL_ENABLED_CAPABILITY, std_msgs.Bool._TYPE);
        antilockBrakeSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + ANTILOCK_BRAKES_ACTIVE_CAPABILITY, std_msgs.Bool._TYPE);
        stabilityActiveSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + STABILITY_CTRL_ACTIVE_CAPABILITY, std_msgs.Bool._TYPE);
        stabilityEnabledSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + STABILITY_CTRL_ENABLED_CAPABILITY, std_msgs.Bool._TYPE);
        parkingBrakeSubscriber = pubSubService.getSubscriberForTopic(CAN_DRIVER_BASE_PATH + PARKING_BRAKE_CAPABILITY, std_msgs.Bool._TYPE);

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

		    try {
			    getTransformClient = pubSubService.getServiceForTopic("get_transform", GetTransform._TYPE);
		    } catch (TopicNotFoundException tnfe) {
			    exceptionHandler.handleException("get_transform service cannot be found", tnfe);
		    }

        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onActive() {
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

        currentState.set(GuidanceState.ACTIVE);
    }
    
    @Override
    public void onDeactivate() {
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
	public void onShutdown() {
		super.onShutdown();
		if(getTransformClient != null) {
		    getTransformClient.close();
		}
	}

	@Override
	public void onPanic() {
		super.onPanic();
		if(getTransformClient != null) {
		    getTransformClient.close();
		}
	}
	
	@Override
	public void timingLoop() throws InterruptedException {
		
		long loop_start = System.currentTimeMillis();
		
		if(currentState.get() != GuidanceState.STARTUP && currentState.get() != GuidanceState.SHUTDOWN) {
			
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
		if(navSatFixSubscriber != null && navSatFixSubscriber.getLastMessage() != null) {
			NavSatFix gps_msg = navSatFixSubscriber.getLastMessage();
			Transform earthToHostVehicle = getTransform("earth", "host_vehicle", gps_msg.getHeader().getStamp());

			if (earthToHostVehicle == null) {
				// No transform so leave the lat/lon/elev marked as unavailable
				log.info("TRANSFORM", "Could not get transform for BSM");
			} else {
				GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
				Location loc = gcc.cartesian2Geodesic(new Point3D(0,0,0), earthToHostVehicle);
				double lat = loc.getLatitude();
				double Lon = loc.getLongitude();
				float elev = (float) loc.getAltitude();
				if(lat >= BSMCoreData.LATITUDE_MIN && lat <= BSMCoreData.LATITUDE_MAX) {
					coreData.setLatitude(lat);
					coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.LATITUDE_AVAILABLE));
				}
				if(Lon >= BSMCoreData.LONGITUDE_MIN && Lon <= BSMCoreData.LONGITUDE_MAX) {
					coreData.setLongitude(Lon);
					coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.LONGITUDE_AVAILABLE));
				}
				if(elev >= BSMCoreData.ELEVATION_MIN && elev <= BSMCoreData.ELEVATION_MAX) {
					coreData.setElev(elev);
					coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.ELEVATION_AVAILABLE));
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
				coreData.getAccuracy().setPresenceVector((short)(coreData.getAccuracy().getPresenceVector() | PositionalAccuracy.ACCURACY_AVAILABLE));
			}
			if(semi_minor != -1) {
				if(semi_minor >= PositionalAccuracy.ACCURACY_MAX) {
					coreData.getAccuracy().setSemiMinor(PositionalAccuracy.ACCURACY_MAX);
				} else {
					coreData.getAccuracy().setSemiMinor(semi_minor);
				}
				coreData.getAccuracy().setPresenceVector((short)(coreData.getAccuracy().getPresenceVector() | PositionalAccuracy.ACCURACY_AVAILABLE));
			}
			
			if(orientation >= PositionalAccuracy.ACCURACY_ORIENTATION_MIN && orientation <= PositionalAccuracy.ACCURACY_ORIENTATION_MAX) {
				coreData.getAccuracy().setOrientation(orientation);
				coreData.getAccuracy().setPresenceVector((short)(coreData.getAccuracy().getPresenceVector() | PositionalAccuracy.ACCURACY_ORIENTATION_AVAILABLE));
			}
		}
		
		// Set transmission state
		// Reserved values are illegal values at this time
		coreData.getTransmission().setTransmissionState(j2735_msgs.TransmissionState.UNAVAILABLE);
		if(transmissionSubscriber != null && transmissionSubscriber.getLastMessage() != null) {
			byte transmission_state = transmissionSubscriber.getLastMessage().getTransmissionState();
			if(transmission_state == j2735_msgs.TransmissionState.NEUTRAL
					|| transmission_state == j2735_msgs.TransmissionState.FORWARDGEARS
					|| transmission_state == j2735_msgs.TransmissionState.PARK
					|| transmission_state == j2735_msgs.TransmissionState.REVERSEGEARS) {
				coreData.getTransmission().setTransmissionState(transmission_state);
			}
		}

		if(velocity_ready.get()) {
			float speed = (float) current_speed.get();
                        if(speed < BSMCoreData.SPEED_MIN) {
                            coreData.setSpeed(0);
                        } else if(speed >= BSMCoreData.SPEED_MIN && speed <= BSMCoreData.SPEED_MAX) {
				coreData.setSpeed(speed);
				coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.SPEED_AVAILABLE));
			}
		}
		
		if(headingStampedSubscriber != null && headingStampedSubscriber.getLastMessage() != null) {
			float heading = (float) headingStampedSubscriber.getLastMessage().getHeading();
			if(heading >= BSMCoreData.HEADING_MIN && heading <= BSMCoreData.HEADING_MAX) {
				coreData.setHeading(heading);
				coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.HEADING_AVAILABLE));
			}
		}
		
		if(steeringWheelSubscriber != null && steeringWheelSubscriber.getLastMessage() != null) {
			float angle = (float) steeringWheelSubscriber.getLastMessage().getData();	
			if(angle <= BSMCoreData.STEER_WHEEL_ANGLE_MIN) {
				coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_MIN);
			} else if(angle >= BSMCoreData.STEER_WHEEL_ANGLE_MAX) {
				coreData.setAngle(BSMCoreData.STEER_WHEEL_ANGLE_MAX);
			} else {
				coreData.setAngle(angle);
			}
			coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.STEER_WHEEL_ANGLE_AVAILABLE));
		}

		// TODO: It is not well defined in J2735
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

			coreData.getAccelSet().setPresenceVector((short)(coreData.getAccelSet().getPresenceVector() | AccelerationSet4Way.ACCELERATION_AVAILABLE));
			
			if(accel_set_linear.getY() <= AccelerationSet4Way.ACCELERATION_MIN) {
				acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MIN);
			} else if(accel_set_linear.getY() >= AccelerationSet4Way.ACCELERATION_MAX) {
				acceleration.setLateral(AccelerationSet4Way.ACCELERATION_MAX);
			} else {
				acceleration.setLateral((float) accel_set_linear.getY());
			}

			coreData.getAccelSet().setPresenceVector((short)(coreData.getAccelSet().getPresenceVector() | AccelerationSet4Way.ACCELERATION_AVAILABLE));

			if(accel_set_linear.getZ() <= AccelerationSet4Way.ACCELERATION_VERTICAL_MIN) {
				acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MIN);
			} else if(accel_set_linear.getZ() >= AccelerationSet4Way.ACCELERATION_VERTICAL_MAX) {
				acceleration.setVert(AccelerationSet4Way.ACCELERATION_VERTICAL_MAX);
			} else {
				acceleration.setVert((float) accel_set_linear.getZ());
			}

			coreData.getAccelSet().setPresenceVector((short)(coreData.getAccelSet().getPresenceVector() | AccelerationSet4Way.ACCELERATION_VERTICAL_AVAILABLE));
			
			double yaw_rate = accelerationSubscriber.getLastMessage().getAccel().getAngular().getZ();
			if(yaw_rate >= AccelerationSet4Way.YAWRATE_MIN && yaw_rate <= AccelerationSet4Way.YAWRATE_MAX) {
				coreData.getAccelSet().setYawRate((float) yaw_rate);
			}

			coreData.getAccelSet().setPresenceVector((short)(coreData.getAccelSet().getPresenceVector() | AccelerationSet4Way.YAWRATE_AVAILABLE));
		}
		
		coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_STATUS_UNAVAILABLE);
		if(brakeSubscriber != null && brakeSubscriber.getLastMessage() != null) {
			if(brakeSubscriber.getLastMessage().getData() > BRAKES_NOT_APPLIED) {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_APPLIED);
			} else {
				coreData.getBrakes().getWheelBrakes().setBrakeAppliedStatus(BRAKES_NOT_APPLIED);
			}
		}
		
		
		coreData.getBrakes().getTraction().setTractionControlStatus(j2735_msgs.TractionControlStatus.UNAVAILABLE);
		coreData.getBrakes().getAbs().setAntiLockBrakeStatus(j2735_msgs.AntiLockBrakeStatus.UNAVAILABLE);
		coreData.getBrakes().getScs().setStabilityControlStatus(j2735_msgs.StabilityControlStatus.UNAVAILABLE);
		coreData.getBrakes().getAuxBrakes().setAuxiliaryBrakeStatus(j2735_msgs.AuxiliaryBrakeStatus.UNAVAILABLE);
		// TODO: N/A for now
		coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(j2735_msgs.BrakeBoostApplied.UNAVAILABLE);
		
		if(tractionActiveSubscriber != null && tractionActiveSubscriber != null && tractionActiveSubscriber.getLastMessage() != null && tractionActiveSubscriber.getLastMessage().getData()) {
			if(tractionEnabledSubscriber != null && tractionEnabledSubscriber != null && tractionEnabledSubscriber.getLastMessage() != null && tractionEnabledSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getTraction().setTractionControlStatus(j2735_msgs.TractionControlStatus.ENGAGED);
			} else {
				coreData.getBrakes().getTraction().setTractionControlStatus(j2735_msgs.TractionControlStatus.ON);
			}
		}
		if(antilockBrakeSubscriber != null && antilockBrakeSubscriber != null && antilockBrakeSubscriber.getLastMessage() != null && antilockBrakeSubscriber.getLastMessage().getData()) {
			coreData.getBrakes().getAbs().setAntiLockBrakeStatus(j2735_msgs.AntiLockBrakeStatus.ON);
		}
		if(stabilityActiveSubscriber != null && stabilityActiveSubscriber.getLastMessage() != null && stabilityActiveSubscriber.getLastMessage().getData()) {
			if(stabilityEnabledSubscriber != null && stabilityEnabledSubscriber.getLastMessage() != null && stabilityEnabledSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getScs().setStabilityControlStatus(j2735_msgs.StabilityControlStatus.ENGAGED);
			} else {
				coreData.getBrakes().getScs().setStabilityControlStatus(j2735_msgs.StabilityControlStatus.ON);
			}
		}
		if(parkingBrakeSubscriber != null && parkingBrakeSubscriber.getLastMessage() != null) {
			if(parkingBrakeSubscriber.getLastMessage().getData()) {
				coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(j2735_msgs.BrakeBoostApplied.ON);
			} else {
				coreData.getBrakes().getBrakeBoost().setBrakeBoostApplied(j2735_msgs.BrakeBoostApplied.OFF);
			}
		}
		
		// Set length and width only for the first time
		if(vehicleLength >= VehicleSize.VEHICLE_LENGTH_MIN && vehicleLength <= VehicleSize.VEHICLE_LENGTH_MAX) {
			coreData.getSize().setVehicleLength(vehicleLength);
			coreData.getSize().setPresenceVector((short)(coreData.getSize().getPresenceVector() | VehicleSize.VEHICLE_LENGTH_AVAILABLE));
		}
		if(vehicleWidth >= VehicleSize.VEHICLE_WIDTH_MIN && vehicleWidth <= VehicleSize.VEHICLE_WIDTH_MAX) {
			coreData.getSize().setVehicleWidth(vehicleWidth);
			coreData.getSize().setPresenceVector((short)(coreData.getSize().getPresenceVector() | VehicleSize.VEHICLE_WIDTH_AVAILABLE));
		}
		
		// Use ros node time and ignore leap second for now since it is not announced
		coreData.setSecMark((short) (System.currentTimeMillis() % BSMCoreData.SEC_MARK_MOD));
		coreData.setPresenceVector((short)(coreData.getPresenceVector() | BSMCoreData.SEC_MARK_AVAILABLE));

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
            jobQueue.add(this::onActive);
            break;
        case DEACTIVATE:
            jobQueue.add(this::onDeactivate);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
    	case PANIC_SHUTDOWN:
     		jobQueue.add(this::onPanic);
      		break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }

    private Transform getTransform(String parentFrame, String childFrame, Time stamp) {
	    GetTransformRequest request = getTransformClient.newMessage();
	    request.setParentFrame(parentFrame);
	    request.setChildFrame(childFrame);
	    request.setStamp(stamp);

	    final GetTransformResponse[] response = new GetTransformResponse[1];
	    final boolean[] gotTransform = {false};

	    getTransformClient.call(request,
		    new OnServiceResponseCallback<GetTransformResponse>() {

			    @Override
			    public void onSuccess(GetTransformResponse msg) {
			    	if (msg.getErrorStatus() == GetTransformResponse.NO_ERROR
					    || msg.getErrorStatus() == GetTransformResponse.COULD_NOT_EXTRAPOLATE) {
					    response[0] = msg;
					    gotTransform[0] = true;
				    } else {
			    		log.warn("TRANSFORM", "Request for transform ParentFrame: " + request.getParentFrame() +
						    " ChildFrame: " + request.getChildFrame() + " returned ErrorCode: " + msg.getErrorStatus());
				    }
			    }

			    @Override
			    public void onFailure(Exception e) {
				    exceptionHandler.handleException("get_transform service call failed", e);
			    }

		    });

	    if(gotTransform[0]) {
	    	return Transform.fromTransformMessage(response[0].getTransform().getTransform());
	    } else {
	    	return null;
	    }
    }

    @Override
    public String getCurrentBSMId() {
        if(random_id != null) {
            char[] hexChars = new char[random_id.length * 2];
            for(int i = 0; i < random_id.length; i++) {
                int firstFourBits = (0xF0 & random_id[i]) >>> 4;
                int lastFourBits = 0xF & random_id[i];
                hexChars[i * 2] = Integer.toHexString(firstFourBits).charAt(0);
                hexChars[i * 2 + 1] = Integer.toHexString(lastFourBits).charAt(0);
            }
            return new String(hexChars);
        }
        return "00000000";
    }
}
