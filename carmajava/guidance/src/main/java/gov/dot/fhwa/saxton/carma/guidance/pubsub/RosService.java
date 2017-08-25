package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class RosService<T, S> implements IService<T, S> {
    RosService(ServiceClient<T, S> serviceClient, RosServiceChannel<T, S> parent) {
        this.serviceClient = serviceClient;
        this.parent = parent;
    }

    @Override public void call(T request, final OnServiceResponseCallback<S> callback) {
        if (open) {
            serviceClient.call(request, new ServiceResponseListener<S>() {
                @Override public void onSuccess(S s) {
                    callback.onSuccess(s);
                }

                @Override public void onFailure(RemoteException e) {
                    callback.onFailure(e);
                }
            });
        }
    }

    @Override public void close() {
        parent.notifyClientShutdown();
    }

    protected ServiceClient<T, S> serviceClient;
    protected RosServiceChannel<T, S> parent;
    protected boolean open = true;
}
