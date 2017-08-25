package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface ISubscriptionChannel<T> {
    ISubscriber<T> getSubscriber();
    boolean isOpen();
    void close();
    void notifyClientShutdown();
}
