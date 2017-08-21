package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface OnMessageCallback<T> {
    /**
     * A callback intended to be invoked when an IService or ISubscriptionChannel receives a new message
     * @param msg The message received
     */
    void onMessage(T msg);
}
