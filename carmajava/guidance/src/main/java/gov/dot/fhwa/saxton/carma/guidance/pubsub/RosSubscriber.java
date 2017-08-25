package gov.dot.fhwa.saxton.carma.guidance.pubsub;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

public class RosSubscriber<T> implements ISubscriber<T> {
    protected Subscriber<T> subscriber;
    protected RosSubscriptionChannel<T> parent;
    protected T lastMessage = null;

    RosSubscriber(Subscriber<T> subscriber, RosSubscriptionChannel<T> parent) {
        this.subscriber = subscriber;
        this.parent = parent;
        subscriber.addMessageListener(new MessageListener<T>() {
            @Override public void onNewMessage(T t) {
                lastMessage = t;
            }
        });
    }

    @Override public T getLastMessage() {
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
        parent.notifyClientShutdown();
    }
}
