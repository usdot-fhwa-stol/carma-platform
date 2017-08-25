package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.service.ServiceClient;

public class RosServiceChannel<T, S> implements IServiceChannel<T, S> {
    RosServiceChannel(ServiceClient<T, S> serviceClient) {
        this.serviceClient = serviceClient;
    }

    /**
     * Get a new IService instance for this topic
     */
    @Override public IService<T, S> getService() {
        numOpenChannels++;

        return new RosService<>(serviceClient, this);
    }

    /**
     * Notice an IService instance being closed. If there are no remaining instances shut down the resource.
     */
    @Override public void notifyClientShutdown() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            close();
        }
    }

    /**
     * Return whether or not the underlying resource for this RosPublicationChannel has been shut down
     */
    @Override public boolean isOpen() {
        return open;
    }

    @Override public void close() {
        open = false;
    }

    /**
     * Get the number of extant IService instances that haven't been closed
     */
    public int getNumOpenChannel() {
        return numOpenChannels;
    }

    protected int numOpenChannels = 0;
    protected boolean open = true;
    protected ServiceClient<T, S> serviceClient;
}
