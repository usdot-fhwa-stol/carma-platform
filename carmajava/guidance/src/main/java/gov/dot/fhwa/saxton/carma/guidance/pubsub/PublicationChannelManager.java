package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


/**
 * Manages the underlying resources associated with a given ROS topic publication
 * @param <T>
 */
public class PublicationChannelManager<T> {
    PublicationChannelManager(ConnectedNode node, String topicUrl, String type) {
        this.publisher = node.newPublisher(topicUrl, type);
    }

    /**
     * Get a new IPublicationChannel instance for the topic
     */
    IPublicationChannel<T> getNewChannel() {
        numOpenChannels++;
        return new RosPublicationChannel<>(publisher, this);
    }

    /**
     * Notice that an instance has been closed. If there are no more instances shut down the resource
     */
    void registerChannelDestroy() {
        numOpenChannels--;

        if (numOpenChannels <= 0) {
            publisher.shutdown();
        }
    }

    /**
     * Get the nubmer of extant IPublicationChannel instances that haven't been closed
     */
    public int getNumOpenChannels() {
        return numOpenChannels;
    }

    protected int numOpenChannels = 0;
    protected Publisher<T> publisher;
}
