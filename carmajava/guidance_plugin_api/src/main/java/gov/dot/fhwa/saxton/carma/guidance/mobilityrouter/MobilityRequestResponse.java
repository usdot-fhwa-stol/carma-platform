package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

/**
 * An {@link IPlugin}'s response to an {@link MobilityRequest} message.
 * <p>
 * ACK - indicates that the plugin accepts the MobilityRequest and will handle making any adjustments needed to avoid a conflict
 * NACK - indicates that the plugin rejects the MobilityRequest and would suggest the other vehicle replan
 * NO_RESPONSE - indicates that the plugin is indifferent but sees no conflict
 */
public enum MobilityRequestResponse {
    ACK,
    NACK,
    NO_RESPONSE
}