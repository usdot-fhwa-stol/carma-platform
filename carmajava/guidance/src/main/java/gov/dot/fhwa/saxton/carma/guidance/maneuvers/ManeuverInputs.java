package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import cav_msgs.RoadwayEnvironment;
import cav_msgs.Route;
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


    ManeuverInputs(AtomicReference<GuidanceState> state, IPubSubService iPubSubService, ConnectedNode node) {
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
        responseLag_ = params.getDouble("vehicle_response_lag");
        timeStep_ = params.getInteger("time_step", 100);


        while (!Thread.currentThread().isInterrupted()) {
        }
    }


    @Override
    public void onSystemReady() {
    }


    @Override
    public void onGuidanceEnable() {
    }


    @Override
    public void loop() {
    }


    @Override
    public String getComponentName() {
        return "guidance.maneuvers.ManeuverInputs";
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
    public int getTimeStep() {
        return timeStep_;
    }
}
