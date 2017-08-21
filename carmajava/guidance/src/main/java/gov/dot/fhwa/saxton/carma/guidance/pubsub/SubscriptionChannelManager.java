package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class SubscriptionChannelManager<T> {
    SubscriptionChannelManager(ConnectedNode node, String topicUrl, String type) {
        this.subscriber = node.newSubscriber(topicUrl, type);
    }

    public ISubscriptionChannel<T> getNewChannel() {
        numOpenChannels++;
        return new RosSubscriptionChannel<>(subscriber, this);
    }

    public void registerChannelDestroy() {
        numOpenChannels--;
    }

    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    protected int numOpenChannels = 0;
    protected Subscriber<T> subscriber;
}
