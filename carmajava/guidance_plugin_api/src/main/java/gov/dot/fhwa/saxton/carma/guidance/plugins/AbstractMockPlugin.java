package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;

public abstract class AbstractMockPlugin extends AbstractPlugin {
    public AbstractMockPlugin(IPubSubService pubSubService) {
        this.pubSubService = pubSubService;
    }

    @Override public void onInitialize() {
        publisher = pubSubService.getPublisherForTopic(topicPrefix + getName() + getVersionId(), SystemAlert._TYPE);
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    @Override public void onResume() {
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    private void computeAvailability() {
        availability = Math.random() > 0.5;
    }

    @Override public void loop() throws InterruptedException {
        if (getActivation()) {
            computeAvailability();

            SystemAlert msg = publisher.newMessage();
            msg.setType(SystemAlert.CAUTION);
            msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " looping. Avail=" + availability);
            publisher.publish(msg);
        }

        Thread.sleep(sleepDuration);
    }

    @Override public void onSuspend() {
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    @Override public void onTerminate() {
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    protected IPubSubService pubSubService;
    protected IPublisher<SystemAlert> publisher;
    protected final String topicPrefix = "/guidance/plugins/";
    protected final long sleepDuration = 30000;
}
