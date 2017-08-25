package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;

public class RosServiceChannelFactory implements IServiceChannelFactory {
    public RosServiceChannelFactory(ConnectedNode node) {
        this.node = node;
    }

    @Override public <T, S> IServiceChannel<T, S> newServiceChannel(String topic, String type) throws TopicNotFoundException {
        try {
            return (IServiceChannel<T, S>) new RosServiceChannel<>(node.newServiceClient(topic, type));
        } catch (ServiceNotFoundException e) {
            throw new TopicNotFoundException();
        }
    }

    protected ConnectedNode node;
}
