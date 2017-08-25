package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface ISubscriber<T> {
    /**
     * Get the last message received on this channel if one has been received.
     *
     * @return Either the last received message or null if none has been received yet
     */
    T getLastMessage();

    /**
     * Add a callback to be executed whenever a new message is received for this ISubscriptionChannel
     *
     * @param callback The callback to be executed
     */
    void registerOnMessageCallback(OnMessageCallback<T> callback);

    /**
     * Notify this ISubscriptionChannel instance's parent of this instance's closure. This will not necessarily close
     * the underlying resources associated with this IService.
     */
    void close();
}
