package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


public class PublicationChannelManager<T> {
    PublicationChannelManager(ConnectedNode node, String topicUrl, String type) {
        this.publisher = node.newPublisher(topicUrl, type);
    }

    public IPublicationChannel<T> getNewChannel() {
        numOpenChannels++;
        return new RosPublicationChannel<>(publisher, this);
    }

    public void registerChannelDestroy() {
        numOpenChannels--;
    }

    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    protected int numOpenChannels = 0;
    protected Publisher<T> publisher;
}
