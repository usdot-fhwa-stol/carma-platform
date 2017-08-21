package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

public class ServiceManager<T, S> {
    ServiceManager(ConnectedNode node, String topicUrl, String typeName) {
    }

    public void openServiceClient() throws ServiceNotFoundException {
        serviceClient = node.newServiceClient(topicUrl, typeName);
    }

    public IService<T, S> getNewChannel() {
        numOpenChannels++;

        return new RosService<>(serviceClient, this);
    }

    public void registerServiceClose() {
        numOpenChannels--;
    }

    public int getNumOpenChannel() {
        return numOpenChannels;
    }

    protected String topicUrl;
    protected String typeName;
    protected ServiceClient<T, S> serviceClient;
    protected ConnectedNode node;
    protected int numOpenChannels = 0;
}
