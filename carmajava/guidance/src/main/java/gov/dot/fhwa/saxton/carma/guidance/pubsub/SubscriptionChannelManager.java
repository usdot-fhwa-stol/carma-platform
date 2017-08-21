package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * Package private class for use in the PubSubManager
 * <p>
 * Responsible for keeping track of the subscription channel resources associated with any number of ISubscriptionChannels
 * for a given topic.
 *
 * @param <T> Type parameter for the message of the topic
 */
public class SubscriptionChannelManager<T> {
    SubscriptionChannelManager(ConnectedNode node, String topicUrl, String type) {
        this.subscriber = node.newSubscriber(topicUrl, type);
    }

    /**
     * Acquire a new ISubscriptionChannel instance
     */
    ISubscriptionChannel<T> getNewChannel() {
        numOpenChannels++;
        return new RosSubscriptionChannel<>(subscriber, this);
    }

    /**
     * Register the destruction of a channel interface instance. If none are open then close the resource.
     */
    void registerChannelDestroy() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            subscriber.shutdown();
        }
    }

    /**
     * Get the number of extant channel instances that haven't been closed yet
     */
    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    /**
     * Return whether or not the underlying resource for this PublicationChannelManager has been shut down
     */
    public boolean isOpen() {
        return open;
    }

    protected int numOpenChannels = 0;
    protected Subscriber<T> subscriber;
    protected boolean open = true;
}
