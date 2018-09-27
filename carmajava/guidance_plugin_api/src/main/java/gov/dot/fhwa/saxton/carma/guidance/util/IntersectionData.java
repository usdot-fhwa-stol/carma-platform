package gov.dot.fhwa.saxton.carma.guidance.util;

import java.time.LocalDateTime;

import cav_msgs.IntersectionGeometry;
import cav_msgs.IntersectionState;

/**
 * Intersection data container object
 * <p>
 * IntersectionData contains the geometry and signal phase data for a single interesection detected
 * via DSRC. IntersectionData may contain only a MAP message if that is all that has been received but
 * will never contain less than only a MAP message.
 */
public class IntersectionData {
    private IntersectionGeometry geometry;
    private LocalDateTime mapRxTimestamp;
    private IntersectionState state;
    private LocalDateTime stateRxTimestamp;

    private int intersectionId;

    /**
     * Construct an IntersectionData object with only geometry
     */
    public IntersectionData(IntersectionGeometry geometry, LocalDateTime mapRxTimestamp) {
        this.geometry = geometry;
        this.mapRxTimestamp = mapRxTimestamp;

        intersectionId = geometry.getId().getId();
    }

    /**
     * Construct an IntersectionData object with only geometry, using inferred current time as RX timestamp
     */
    public IntersectionData(IntersectionGeometry geometry) {
        this.geometry = geometry;
        this.mapRxTimestamp = LocalDateTime.now();

        intersectionId = geometry.getId().getId();
    }

    /**
     * Construct an IntersectionData object with both geometry and state
     */
    public IntersectionData(IntersectionGeometry geometry, LocalDateTime mapRxTimestamp, IntersectionState state, LocalDateTime stateRxTimestamp) {
        this(geometry, mapRxTimestamp);
        this.state = state;
        this.stateRxTimestamp = stateRxTimestamp;
    }

    /**
     * Construct an IntersectionData object with both geometry and state, using inferred current time as RX timestamp for both
     */
    public IntersectionData(IntersectionGeometry geometry, IntersectionState state) {
        this(geometry);
        this.state = state;
        this.stateRxTimestamp = LocalDateTime.now();
    }

    /**
     * Get the DSRC ID associated with the intersection
     */
    public int getIntersectionId() {
        return intersectionId;
    }

    /**
     * Check if this object has intersection state data suitable for consumption
     */
    public boolean hasIntersectionState() {
        return state != null;
    }

    /**
     * Get the current intersection state for this interesction
     */
    public IntersectionState getIntersectionState() {
        return state;
    }

    /**
     * Package-private member to update intersection state for this instance, immutable outside of this pacakge.
     */
    protected void updateIntersectionState(IntersectionState state, LocalDateTime timestamp) {
        if (state.getId().getId() != intersectionId) {
            // Do not update as this SPAT is not for this intersection, 
            return;
        }

        this.state = state;
        this.stateRxTimestamp = timestamp;
    }

    /**
     * Package-private member to update intersection geometry for this instance, immutable outside of this pacakge.
     */
    protected void updateIntersectionGeometry(IntersectionGeometry geometry, LocalDateTime timestamp) {
        this.geometry = geometry;
        this.stateRxTimestamp = timestamp;
    }

    /**
     * Get the timestamp the message containing this intersection state data was received
     */
    public LocalDateTime getIntersectionStateRxTimestamp() {
        return stateRxTimestamp;
    }

    /**
     * Get the current intersection geometry for this interesection
     */
    public IntersectionGeometry getIntersectionGeometry() {
        return geometry;
    }

    /**
     * Get the timestamp the message containing this intersection geometry data was received
     */
    public LocalDateTime getIntersectionGeometryRxTimestamp() {
        return mapRxTimestamp;
    }
}