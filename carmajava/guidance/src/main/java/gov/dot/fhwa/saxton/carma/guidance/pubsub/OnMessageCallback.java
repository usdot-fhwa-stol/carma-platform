package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface OnMessageCallback<T> {
    void onMessage(T msg);
}
