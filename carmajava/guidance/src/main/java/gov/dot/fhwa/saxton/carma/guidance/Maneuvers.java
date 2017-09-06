package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.Maneuver;
import cav_msgs.NewPlan;
import cav_msgs.RoadwayEnvironment;
import cav_msgs.Route;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;

/**
 * Guidance sub-component responsible for the execution of maneuvers and generation of maneuver plans
 */
public class Maneuvers implements Runnable {
    Maneuvers(IPubSubService iPubSubService, ConnectedNode node) {
        this.iPubSubService = iPubSubService;
        this.node = node;
        this.log = node.getLog();
    }

    @Override public void run() {
        // Set up subscribers
        ISubscriber<RoadwayEnvironment> roadwayEnvironmentSubscriber =
            iPubSubService.getSubscriberForTopic("roadway_environment",
                RoadwayEnvironment._TYPE);

        roadwayEnvironmentSubscriber.registerOnMessageCallback(new OnMessageCallback<RoadwayEnvironment>() {
            @Override public void onMessage(RoadwayEnvironment msg) {
                log.info("Received RoadwayEnvironment:" + msg);
            }
        });

        ISubscriber<Route> routeSubscriber = iPubSubService.getSubscriberForTopic("route",
            Route._TYPE);

        routeSubscriber.registerOnMessageCallback(new OnMessageCallback<Route>() {
            @Override public void onMessage(Route msg) {
                log.info("Received Route:" + msg);
            }
        });

        // Set up publishers
        IPublisher<NewPlan> newPlanPublisher = iPubSubService
            .getPublisherForTopic("new_plan", NewPlan._TYPE);

        while (!Thread.currentThread().isInterrupted()) {
            NewPlan newPlan = newPlanPublisher.newMessage();
            newPlan.setExpiration(node.getCurrentTime());
            newPlan.getHeader().setChecksum((short) 0);
            newPlan.getHeader().setPlanId(curPlanId++);
            newPlan.getHeader().setSenderId("test-cav1");
            newPlan.getHeader().setRecipientId("test-cav2");
            newPlan.setObjective((byte) 3);

            List<String> participantIds = new ArrayList<>();
            participantIds.add("test-cav1");
            participantIds.add("test-cav2");
            newPlan.setParticipantIds(participantIds);

            List<cav_msgs.Maneuver> maneuvers = new ArrayList<>();
            Maneuver m  = node.getTopicMessageFactory().newFromType(Maneuver._TYPE);
            m.setLength(10);
            m.setPerformers(2);
            m.setStartRoadwayLaneId((byte) 0);
            m.setStartRoadwayLink("link1");
            m.setStartRoadwayOriginatorPosition(0);
            m.setType((byte) 8);
            maneuvers.add(m);
            newPlan.setManeuvers(maneuvers);

            newPlanPublisher.publish(newPlan);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
                e.printStackTrace();
                Thread.currentThread().interrupt(); // Rethrow the interrupt
            }
        }
    }

    protected IPubSubService iPubSubService;
    protected ConnectedNode node;
    protected Log log;
    protected long sleepDurationMillis = 30000;
    protected short curPlanId = 0;
}
