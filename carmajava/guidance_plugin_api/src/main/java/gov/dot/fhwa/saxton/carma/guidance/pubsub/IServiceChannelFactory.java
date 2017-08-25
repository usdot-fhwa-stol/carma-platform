package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means to create new {@link IServiceChannel} instances for new topics/services
 */
public interface IServiceChannelFactory {
    /**
     * Get a new {@link IServiceChannel} instance for the specified topic and type
     * @param topic The URL for the service topic
     * @param type The String identifier for the service dialogue type
     * @param <T> Type parameter for the service request message
     * @param <S> Type parameter for the service response message
     * @throws TopicNotFoundException If there is not a valid service residing at the specified URL
     */
    <T, S> IServiceChannel<T, S> newServiceChannel(String topic, String type) throws TopicNotFoundException;
}
