package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

public interface IMobilityRouter {
    public void registerMobilityRequestHandler(String strategy, MobilityRequestHandler handler);

    public void registerMobilityAckHandler(String strategy, MobilityAckHandler handler);

    public void registerMobilityOperationHandler(String strategy, MobilityOperationHandler handler);

    public void registerMobilityPathHandler(String strategy, MobilityPathHandler handler);
}