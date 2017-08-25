package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means by which to share an underyling resource for a topic
 *
 * @param <T>
 */
public interface ISubscriptionChannel<T> {
    /**
     * Get an {@link ISubscriber} instance for the configured topic
     */
    ISubscriber<T> getSubscriber();

    /**
     * Check if the underlying resource is still available
     */
    boolean isOpen();

    /**
     * Close the underlying resources
     */
    void close();

    /**
     * Receive notification from a client that it no longer needs the underlying resource. If no
     * more clients are active, close the resource.
     */
    void notifyClientShutdown();
}
