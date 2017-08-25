package gov.dot.fhwa.saxton.carma.guidance.pubsub;

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
