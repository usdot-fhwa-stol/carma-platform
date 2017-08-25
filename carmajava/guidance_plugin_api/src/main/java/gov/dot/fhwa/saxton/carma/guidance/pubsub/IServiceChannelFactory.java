package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IServiceChannelFactory {
    <T, S> IServiceChannel<T, S> newServiceChannel(String topic, String type) throws TopicNotFoundException;
}
