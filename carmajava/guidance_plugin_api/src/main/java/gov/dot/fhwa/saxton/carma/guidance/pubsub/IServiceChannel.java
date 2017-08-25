package gov.dot.fhwa.saxton.carma.guidance.pubsub;

public interface IServiceChannel<T, S> {
    IService<T, S> getService();
    void notifyClientShutdown();
    boolean isOpen();
    void close();
}
