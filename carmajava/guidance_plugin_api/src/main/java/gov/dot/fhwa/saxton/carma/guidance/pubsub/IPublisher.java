package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IPublisher<T> {
    /**
     * Generate a new (empty) message instance to have it's fields populated before transmission.
     *
     * @return An empty message of type T
     */
    T newMessage();

    /**
     * Publish a message on this channel
     *
     * @param msg The message to publish
     */
    void publish(T msg);

    /**
     * Notify this IPublicationChannel instance's parent of this instance's closure. This will not necessarily close the
     * underlying resources associated with this IService.
     */
    void close();
}
