package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

public interface IMobilityRouter {
    /**
     * Register a callback to be executed when a mobility request message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityRequestHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityRequestHandler(String strategy, MobilityRequestHandler handler);

    /**
     * Register a callback to be executed when a mobility ack message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityRequestHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityAckHandler(String strategy, MobilityAckHandler handler);

    /**
     * Register a callback to be executed when a mobility operation message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityRequestHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityOperationHandler(String strategy, MobilityOperationHandler handler);

    /**
     * Register a callback to be executed when a mobility path message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityRequestHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityPathHandler(String strategy, MobilityPathHandler handler);

    /**
     * Get the current vehicle's static mobility ID
     */
    public String getHostMobilityId();
}