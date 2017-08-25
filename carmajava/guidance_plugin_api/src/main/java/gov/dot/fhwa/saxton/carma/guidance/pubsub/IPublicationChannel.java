package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IPublicationChannel<T> {
    IPublisher<T> getPublisher();
    boolean isOpen();
    void close();
    void notifyClientShutdown();
}
