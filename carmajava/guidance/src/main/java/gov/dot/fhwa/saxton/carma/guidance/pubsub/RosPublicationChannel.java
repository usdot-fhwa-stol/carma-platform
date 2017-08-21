package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class RosPublicationChannel<T> implements IPublicationChannel<T> {

    public RosPublicationChannel(Publisher<T> publisher, PublicationChannelManager<T> parent) {
        this.publisher = publisher;
        this.parent = parent;
    }

    @Override
    public T newMessage() {
        return publisher.newMessage();
    }

    @Override
    public void publish(T msg) {
        publisher.publish(msg);
    }

    @Override
    public void close() {
        parent.registerChannelDestroy();
    }

    protected Publisher<T> publisher;
    protected PublicationChannelManager<T> parent;
}
