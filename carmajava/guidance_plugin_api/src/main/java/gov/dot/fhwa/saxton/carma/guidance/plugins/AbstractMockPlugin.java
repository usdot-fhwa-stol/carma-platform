package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

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
        log.info("Plugin " + version.componentName() + ":" + version.revisionString() + "initializing...");
        publisher = pubSubService.getPublisherForTopic(topicPrefix, SystemAlert._TYPE);
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + version.componentName() + ":" + version.revisionString() + " initialized");
        publisher.publish(msg);
    }

    @Override public void onResume() {
        log.info("Plugin " + version.componentName() + ":" + version.revisionString() + " resuming...");
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + version.componentName() + ":" + version.revisionString() + " resumed");
        publisher.publish(msg);
    }

    @Override public void loop() throws InterruptedException {
        log.info("Plugin " + version.componentName() + ":" + version.revisionString() + " iterating...");

        if (getActivation()) {
            computeAvailability();

            SystemAlert msg = publisher.newMessage();
            msg.setType(SystemAlert.CAUTION);
            msg.setDescription(
                "Plugin " + version.componentName() + ":" + version.revisionString() + " looping. Avail=" + getAvailability());
            publisher.publish(msg);
        }

        Thread.sleep(sleepDuration);
    }

    @Override public void onSuspend() {
        log.info("Plugin " + version.componentName() + ":" + version.revisionString() + " suspending...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + version.componentName() + ":" + version.revisionString() + " suspended");
        publisher.publish(msg);
    }

    @Override public void onTerminate() {
        log.info("Plugin " + version.componentName() + ":" + version.revisionString() + " terminating...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + version.componentName() + ":" + version.revisionString() + " terminated");
        publisher.publish(msg);
    }

    private void computeAvailability() {
        boolean availability = Math.random() > 0.5;
        setAvailability(availability);
    }

    @Override public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        // NO-OP
    }
}
