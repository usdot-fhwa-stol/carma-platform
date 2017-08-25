package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IPubSubService {
    @SuppressWarnings("unchecked") <T, S> IService<T, S> getServiceForTopic(String topicUrl, String type) throws TopicNotFoundException;

    @SuppressWarnings("unchecked") <T> ISubscriber<T> getSubscriberForTopic(
        String topicUrl, String type);

    @SuppressWarnings("unchecked") <T> IPublisher<T> getPublisherForTopic(String topicUrl,
        String type);
}
