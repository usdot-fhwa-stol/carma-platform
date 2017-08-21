package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import java.util.Optional;

public interface ISubscriptionChannel<T> {
    /**
     * Get the last message received on this channel if one has been received. Optional will be empty o.w.
     * @return An optional possibly containing the last message
     */
    Optional<T> getLastMessage();

    /**
     * Add a callback to be executed whenever a new message is received for this ISubscriptionChannel
     * @param callback The callback to be executed
     */
    void registerOnMessageCallback(OnMessageCallback<T> callback);

    /**
     * Notify this ISubscriptionChannel instance's parent of this instance's closure. This will not necessarily close
     * the underlying resources associated with this IService.
     */
    void close();
}
