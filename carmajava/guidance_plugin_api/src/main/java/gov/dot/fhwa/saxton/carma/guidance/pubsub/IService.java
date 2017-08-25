package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic interface for supporting calls to a remote service. Designed to share an underlying
 * resource between many clients to avoid duplication.
 *
 * @param <T> Type parameter for service request message
 * @param <S> Type parameter for service response message
 */
public interface IService<T, S> {
    /**
     * Perform an asynchronous call for the service, executing the callback after the response is received or upon
     * call failure.
     *
     * @param request  The message for the request
     * @param callback A callback to be executed upon success or failure of the call
     */
    void call(T request, OnServiceResponseCallback<S> callback);

    /**
     * Notify this IService instance's parent of this instance's closure. This will not necessarily close the underlying
     * resources associated with this IService.
     */
    void close();
}
