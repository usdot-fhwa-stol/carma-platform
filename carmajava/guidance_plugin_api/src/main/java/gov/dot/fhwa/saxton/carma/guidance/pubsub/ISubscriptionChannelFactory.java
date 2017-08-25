package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means to create a new {@link ISubscriptionChannel} for a new topic and type
 */
public interface ISubscriptionChannelFactory {
    /**
     * Create a new {@link ISubscriptionChannel} for the specified topic and type
     *
     * @param topic The URL for the topic to subscribe to
     * @param type  The String identifier for the topic message type
     * @param <T>   Type parameter for the topic message type
     * @return A new ISubscriptionChannel instance configured for the topic
     */
    <T> ISubscriptionChannel<T> newSubscriptionChannel(String topic, String type);
}
