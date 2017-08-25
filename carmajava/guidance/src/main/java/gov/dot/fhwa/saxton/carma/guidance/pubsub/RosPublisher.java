package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.topic.Publisher;

public class RosPublisher<T> implements IPublisher<T> {

    public RosPublisher(Publisher<T> publisher, IPublicationChannel<T> parent) {
        this.publisher = publisher;
        this.parent = parent;
    }

    @Override public T newMessage() {
        return publisher.newMessage();
    }

    @Override public void publish(T msg) {
        publisher.publish(msg);
    }

    @Override public void close() {
        parent.notifyClientShutdown();
    }

    protected Publisher<T> publisher;
    protected IPublicationChannel<T> parent;
}
