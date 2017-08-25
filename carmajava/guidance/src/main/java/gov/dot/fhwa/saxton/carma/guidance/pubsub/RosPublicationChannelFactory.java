package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublicationChannel;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.RosPublicationChannel;
import org.ros.node.ConnectedNode;

public class RosPublicationChannelFactory implements IPublicationChannelFactory {
    public RosPublicationChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T> IPublicationChannel<T> newPublicationChannel(String topic, String type) {
        return (IPublicationChannel<T>) new RosPublicationChannel<>(node.newPublisher(topic, type));
    }

    protected ConnectedNode node;
}
