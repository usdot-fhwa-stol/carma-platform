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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import cav_msgs.RouteState;
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
    protected double                                distanceDowntrack_ = 0.0;   // m
    protected double                                currentSpeed_ = 0.0;        // m/s
    protected double                                responseLag_ = 0.0;         // sec
    protected int                                   timeStep_ = 0;              // ms


    public ManeuverInputs(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
        super(state, iPubSubService, node);
    }


    @Override
    public void onGuidanceStartup() {

        //subscribers
        routeStateSubscriber_ = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
        routeStateSubscriber_.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
            @Override
            public void onMessage(RouteState msg) {
                distanceDowntrack_ = msg.getDownTrack();
            }
        });

        twistSubscriber_ = pubSubService.getSubscriberForTopic("twist_stamped", TwistStamped._TYPE);
        twistSubscriber_.registerOnMessageCallback(new OnMessageCallback<TwistStamped>() {
            @Override
            public void onMessage(TwistStamped msg) {
                currentSpeed_ = msg.getTwist().getLinear().getX();
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
}
