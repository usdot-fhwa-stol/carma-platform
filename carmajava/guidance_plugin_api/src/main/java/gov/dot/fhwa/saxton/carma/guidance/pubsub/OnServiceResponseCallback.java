package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface OnServiceResponseCallback<T> {
    void onSuccess(T msg);
    void onFailure(Exception e);
}
