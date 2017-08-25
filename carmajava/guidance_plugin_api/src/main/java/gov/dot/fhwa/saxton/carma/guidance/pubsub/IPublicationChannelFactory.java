package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means to instantiate new {@link IPublicationChannel} instances for new topics
 */
public interface IPublicationChannelFactory {
    /**
     * Create a new {@link IPublicationChannel} for the specified topic, type, and message type
     * parameter
     *
     * @param topic The URL for the topic to be published to
     * @param type  The String identifier of the message type
     * @param <T>   Type parameter for the message class associated with the type value
     * @return A new instance of IPublicationChannel holding all resources needed for publication
     */
    <T> IPublicationChannel<T> newPublicationChannel(String topic, String type);
}
