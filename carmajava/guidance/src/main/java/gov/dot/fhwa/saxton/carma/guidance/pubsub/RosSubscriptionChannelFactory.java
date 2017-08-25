package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;

public class RosSubscriptionChannelFactory implements ISubscriptionChannelFactory {
    protected ConnectedNode node;

    public RosSubscriptionChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T> ISubscriptionChannel<T> newSubscriptionChannel(String topic, String type) {
        return (ISubscriptionChannel<T>) new RosSubscriptionChannel<>(
            node.newSubscriber(topic, type));
    }
}
