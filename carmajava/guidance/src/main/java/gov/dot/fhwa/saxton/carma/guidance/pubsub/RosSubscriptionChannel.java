package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.topic.Subscriber;

/**
 * Package private class for use in the PubSubManager
 * <p>
 * Responsible for keeping track of the subscription channel resources associated with any number of ISubscriptionChannels
 * for a given topic.
 *
 * @param <T> Type parameter for the message of the topic
 */
public class RosSubscriptionChannel<T> implements ISubscriptionChannel<T> {
    protected int numOpenChannels = 0;
    protected boolean open = true;
    protected Subscriber<T> subscriber;

    RosSubscriptionChannel(Subscriber<T> subscriber) {
        this.subscriber = subscriber;
    }

    /**
     * Acquire a new ISubscriber instance
     */
    @Override public ISubscriber<T> getSubscriber() {
        numOpenChannels++;
        return new RosSubscriber<>(subscriber, this);
    }

    /**
     * Register the destruction of a channel interface instance. If none are open then close the resource.
     */
    @Override public void notifyClientShutdown() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            close();
        }
    }

    /**
     * Get the number of extant channel instances that haven't been closed yet
     */
    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    /**
     * Return whether or not the underlying resource for this RosPublicationChannel has been shut down
     */
    @Override public boolean isOpen() {
        return open;
    }

    @Override public void close() {
        open = false;
        subscriber.shutdown();
    }
}
