package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic interface to serve as a seam point for dependency injection. ROS agnostic implementation
 * of the PubSubManager interface such that it can be distributed separately of the rest of Guidance
 */
public interface IPubSubService {
    /**
     * Get access to a new {@link IService} instance for the specified topic and type
     * @param topicUrl The URL of the service topic to access
     * @param type The String identifier of the service dialog type
     * @param <T> Type parameter for the request type of the service dialog
     * @param <S> Type parameter for the response type of the service dialog
     * @return A new IService instance capable of calling the service
     * @throws TopicNotFoundException
     */
    @SuppressWarnings("unchecked") <T, S> IService<T, S> getServiceForTopic(String topicUrl, String type) throws TopicNotFoundException;

    /**
     * Get access to a new {@link ISubscriber} instance for the specified topic and type
     * @param topicUrl The URL of the topic to subscribe to
     * @param type The String identifier of the topic message type
     * @param <T> Type parameter for the topic message type
     * @return A new ISubscriber instance capable of listening to the topic
     */
    @SuppressWarnings("unchecked") <T> ISubscriber<T> getSubscriberForTopic(
        String topicUrl, String type);

    /**
     * Get access to a new {@link IPublisher} instance for the specified topic and type
     * @param topicUrl The URL of the topic to publish to
     * @param type The String identifier of the topic message type
     * @param <T> Type parameter for the topic message type
     * @return A new IPublisher instance capable of publishing data to the topic
     */
    @SuppressWarnings("unchecked") <T> IPublisher<T> getPublisherForTopic(String topicUrl,
        String type);
}
