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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import cav_msgs.ExternalObject;
import cav_msgs.ExternalObjectList;
import cav_msgs.RouteState;
import com.google.common.util.concurrent.AtomicDouble;
import geometry_msgs.TwistStamped;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.params.RosParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.ros.node.ConnectedNode;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Interacts with the ROS network to provide the necessary input data to the Maneuver classes, allowing the rest
 * of the classes in this package to remain ignorant of ROS.
 */
public class ManeuverInputs extends GuidanceComponent implements IManeuverInputs {

    protected ISubscriber<RouteState>               routeStateSubscriber_;
    protected ISubscriber<TwistStamped>             twistSubscriber_;
    protected ISubscriber<ExternalObjectList>       externalObjectListSubscriber_;
    protected double                                distanceDowntrack_ = 0.0;   // m
    protected double                                currentSpeed_ = 0.0;        // m/s
    protected double                                responseLag_ = 0.0;         // sec
    protected int                                   timeStep_ = 0;              // ms
    protected AtomicDouble                          frontVehicleDistance = new AtomicDouble(IAccStrategy.NO_FRONT_VEHICLE_DISTANCE);
    protected AtomicDouble                          frontVehicleSpeed = new AtomicDouble(IAccStrategy.NO_FRONT_VEHICLE_SPEED);


    public ManeuverInputs(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
        super(state, iPubSubService, node);
    }


    @Override
    public void onGuidanceStartup() {
        // Setup the ACC Strategy factory for use by maneuvers
        double maxAccel = node.getParameterTree().getDouble("~vehicle_acceleration_limit", 2.5);
        double vehicleResponseLag = node.getParameterTree().getDouble("~vehicle_response_lag", 1.4);
        double desiredTimeGap = node.getParameterTree().getDouble("~desired_acc_timegap", 1.0);
        double minStandoffDistance = node.getParameterTree().getDouble("~min_acc_standoff_distance", 5.0);
        BasicAccStrategyFactory accFactory = new BasicAccStrategyFactory(desiredTimeGap, maxAccel, vehicleResponseLag, minStandoffDistance);
        AccStrategyManager.setAccStrategyFactory(accFactory);

        //subscribers
        routeStateSubscriber_ = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        routeStateSubscriber_.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override
            public void onMessage(RouteState msg) {
                distanceDowntrack_ = msg.getDownTrack();
            }
        });

        twistSubscriber_ = pubSubService.getSubscriberForTopic("velocity", TwistStamped._TYPE);
        twistSubscriber_.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override
            public void onMessage(TwistStamped msg) {
                currentSpeed_ = msg.getTwist().getLinear().getX();
            }
        });

        /*
         * Presently this subscriber will be mapped by saxton_cav.launch directly to the front long range object topic
         * provided by the Radar Driver, bypassing sensor fusion and the interface manager. This subscriber depends on
         * sensor frame data (rather than odom frame data) to accurately derive distance values to nearby vehicles.
         * Pending full implementation of Sensor Fusion and Roadway modules it is recommended to change this to depend
         * on whatever vehicle frame data is available in those data publications.
         */

        // TODO: Update to use actual topic provided by sensor fusion
        externalObjectListSubscriber_ = pubSubService.getSubscriberForTopic("objects", ExternalObjectList._TYPE);
        externalObjectListSubscriber_.registerOnMessageCallback(new OnMessageCallback<ExternalObjectList>() {
			@Override
			public void onMessage(ExternalObjectList msg) {
                double closestDistance = Double.POSITIVE_INFINITY;
                ExternalObject frontVehicle = null;
                for (ExternalObject eo : msg.getObjects()) {
                    if (eo.getRelativeLane() == ExternalObject.HOST_LANE) {
                        // We found an object in our lane
                        if (eo.getPose().getPose().getPosition().getX() < closestDistance) {
                            // If its close than our previous best candidate, update our candidate
                            frontVehicle = eo;
                            closestDistance = eo.getPose().getPose().getPosition().getX();
                        }
                    }
                }

                // Store our results
                if (frontVehicle != null) {
                    frontVehicleDistance.set(frontVehicle.getPose().getPose().getPosition().getX());
                    frontVehicleSpeed.set(frontVehicle.getVelocity().getTwist().getLinear().getX());
                } else {
                    frontVehicleDistance.set(IAccStrategy.NO_FRONT_VEHICLE_DISTANCE);
                    frontVehicleSpeed.set(IAccStrategy.NO_FRONT_VEHICLE_SPEED);
                }
			}
        });

        //parameters
        RosParameterSource params = new RosParameterSource(node.getParameterTree());
        responseLag_ = params.getDouble("~vehicle_response_lag");
    }


    @Override
    public void onSystemReady() {
    }


    @Override
    public void onGuidanceEnable() {
    }


    @Override
    public String getComponentName() {
        return "Guidance.Maneuvers.ManeuverInputs";
    }


    @Override
    public double getDistanceFromRouteStart() {
        return distanceDowntrack_;
    }


    @Override
    public double getCurrentSpeed() {
        return currentSpeed_;
    }


    @Override
    public double getResponseLag() {
        return responseLag_;
    }


	@Override
	public double getDistanceToFrontVehicle() {
		return frontVehicleDistance.get();
	}


	@Override
	public double getFrontVehicleSpeed() {
		return frontVehicleSpeed.get();
	}
}
