package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;

/**
 * Base class for Mock plugin implementations
 * <p>
 * Handles logging and message publication for
 */
public abstract class AbstractMockPlugin extends AbstractPlugin {
    protected final String topicPrefix = "system_alert";
    protected final long sleepDuration = 5000;
    protected IPublisher<SystemAlert> publisher;

    public AbstractMockPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
    }

    @Override public void onInitialize() {
        log.info("Plugin " + getName() + ":" + getVersionId() + "initializing...");
        publisher = pubSubService.getPublisherForTopic(topicPrefix, SystemAlert._TYPE);
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    @Override public void onResume() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " resuming...");
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " resumed");
        publisher.publish(msg);
    }

    private void computeAvailability() {
        availability = Math.random() > 0.5;
    }

    @Override public void loop() throws InterruptedException {
        log.info("Plugin " + getName() + ":" + getVersionId() + " iterating...");

        if (getActivation()) {
            computeAvailability();

            SystemAlert msg = publisher.newMessage();
            msg.setType(SystemAlert.CAUTION);
            msg.setDescription(
                "Plugin " + getName() + ":" + getVersionId() + " looping. Avail=" + availability);
            publisher.publish(msg);
        }

        Thread.sleep(sleepDuration);
    }

    @Override public void onSuspend() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " suspending...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " suspended");
        publisher.publish(msg);
    }

    @Override public void onTerminate() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " terminating...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " terminated");
        publisher.publish(msg);
    }
}
