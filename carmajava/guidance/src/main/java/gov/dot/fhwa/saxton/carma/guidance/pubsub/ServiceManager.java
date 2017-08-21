package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

public class ServiceManager<T, S> {
    ServiceManager(ConnectedNode node, String topicUrl, String typeName) {
    }

    /**
     * Invoke the functionality to find a service based on topic and type
     * @throws ServiceNotFoundException If the (topic, type) pair does not locate a valid service
     */
    void openServiceClient() throws ServiceNotFoundException {
        serviceClient = node.newServiceClient(topicUrl, typeName);
    }

    /**
     * Get a new IService instance for this topic
     */
    IService<T, S> getNewChannel() {
        numOpenChannels++;

        return new RosService<>(serviceClient, this);
    }

    /**
     * Notice an IService instance being closed. If there are no remaining instances shut down the resource.
     */
    void registerServiceClose() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            open = false;
            serviceClient.shutdown();
        }
    }

    /**
     * Return whether or not the underlying resource for this PublicationChannelManager has been shut down
     */
    public boolean isOpen() {
        return open;
    }

    /**
     * Get the number of extant IService instances that haven't been closed
     */
    public int getNumOpenChannel() {
        return numOpenChannels;
    }

    protected String topicUrl;
    protected String typeName;
    protected ServiceClient<T, S> serviceClient;
    protected ConnectedNode node;
    protected int numOpenChannels = 0;
    protected boolean open = true;
}
