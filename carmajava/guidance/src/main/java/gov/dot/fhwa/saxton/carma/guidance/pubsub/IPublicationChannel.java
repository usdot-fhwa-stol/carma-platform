package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IPublicationChannel<T> {
    T newMessage();
    void publish(T msg);
    void close();
}
