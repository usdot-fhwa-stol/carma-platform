package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

public interface IMobilityRouter {
    public void registerMobilityRequestHandler(Integer targetPlugin, MobilityRequestHandler handler);

    public void registerMobilityAckHandler(Integer targetPlugin, MobilityAckHandler handler);

    public void registerMobilityOperationHandler(Integer targetPlugin, MobilityOperationHandler handler);

    public void registerMobilityStatusHandler(Integer targetPlugin, MobilityPathHandler handler);
}