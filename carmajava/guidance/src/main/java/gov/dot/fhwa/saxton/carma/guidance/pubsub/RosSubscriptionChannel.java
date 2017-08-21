package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

import java.util.Optional;

public class RosSubscriptionChannel<T> implements ISubscriptionChannel<T> {
    RosSubscriptionChannel(Subscriber<T> subscriber, SubscriptionChannelManager<T> parent) {
        this.subscriber = subscriber;
        this.parent = parent;
        subscriber.addMessageListener((msg) -> {
            lastMessage = Optional.of(msg);
        });
    }

    @Override public Optional<T> getLastMessage() {
        return lastMessage;
    }

    @Override public void registerOnMessageCallback(final OnMessageCallback<T> callback) {
        subscriber.addMessageListener(new MessageListener<T>() {
            @Override public void onNewMessage(T t) {
                callback.onMessage(t);
            }
        });
    }

    @Override public void close() {
        parent.registerChannelDestroy();
    }

    protected Subscriber<T> subscriber;
    protected SubscriptionChannelManager<T> parent;
    protected Optional<T> lastMessage = Optional.empty();
}
