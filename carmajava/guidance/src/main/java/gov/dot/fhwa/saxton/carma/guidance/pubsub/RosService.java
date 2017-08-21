package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class RosService<T, S> implements IService<T, S> {
    RosService(ServiceClient<T, S> serviceClient, ServiceManager<T, S> parent) {
        this.serviceClient = serviceClient;
        this.parent = parent;
    }

    @Override public void call(T request, ServiceResponseListener<S> responseListener) {
        if (open) {
            serviceClient.call(request, responseListener);
        }
    }

    @Override public void close() {
        parent.registerServiceClose();
    }

    protected ServiceClient<T, S> serviceClient;
    protected ServiceManager<T, S> parent;
    protected boolean open = true;
}
