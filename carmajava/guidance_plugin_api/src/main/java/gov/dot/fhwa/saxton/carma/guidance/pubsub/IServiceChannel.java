package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic class for sharing underlying resources between multiple {@link IService} instances
 *
 * @param <T> Type parameter for the request type of the service
 * @param <S> Type parameter for the response type of the service
 */
public interface IServiceChannel<T, S> {
    /**
     * Gets a new {@link IService} instance that can utilize the underlying resource
     *
     * @return
     */
    IService<T, S> getService();

    /**
     * Receive notification that a client no longer needs the resources. If there are no more clients,
     * shut down the underlying resources.
     */
    void notifyClientShutdown();

    /**
     * Check whether the underlying resource is still available
     */
    boolean isOpen();

    /**
     * Close the underlying resource
     */
    void close();
}
