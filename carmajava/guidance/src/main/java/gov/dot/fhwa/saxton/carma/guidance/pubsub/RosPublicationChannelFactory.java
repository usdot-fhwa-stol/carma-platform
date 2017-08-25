package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;

public class RosPublicationChannelFactory implements IPublicationChannelFactory {
    protected ConnectedNode node;

    public RosPublicationChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T> IPublicationChannel<T> newPublicationChannel(String topic, String type) {
        return (IPublicationChannel<T>) new RosPublicationChannel<>(node.newPublisher(topic, type));
    }
}
