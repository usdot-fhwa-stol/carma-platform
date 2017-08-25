package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IPublicationChannelFactory {
    <T> IPublicationChannel<T> newPublicationChannel(String topic, String type);
}
