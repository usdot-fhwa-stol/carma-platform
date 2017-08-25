package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic service result callback
 *
 * @param <T> The expected result type of the service
 */
public interface OnServiceResponseCallback<T> {
    /**
     * Invoked on successful call of the service.
     *
     * @param msg The service's response
     */
    void onSuccess(T msg);

    /**
     * Invoked on failed call of the service
     *
     * @param e The exception that resulted from attempting the service call
     */
    void onFailure(Exception e);
}
