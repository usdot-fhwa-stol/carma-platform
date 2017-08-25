package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface ISubscriptionChannelFactory {
    <T> ISubscriptionChannel<T> newSubscriptionChannel(String topic, String type);
}
