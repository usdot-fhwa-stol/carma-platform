package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.service.ServiceResponseListener;

public interface IService<T, S> {
    void call(T request, ServiceResponseListener<S> callback);
    void close();
}
