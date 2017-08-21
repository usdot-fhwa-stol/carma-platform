package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface ISubscriptionChannel<T> {
    T getLastMessage();
    void registerOnMessageCallback(OnMessageCallback<T> callback);
    void close();
}
