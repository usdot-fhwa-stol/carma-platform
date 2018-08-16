package gov.dot.fhwa.saxton.carma.guidance.util;

import java.util.List;

/**
 * Functional interface for receiving notifications only when the list
 * of known intersections changes.
 */
public interface IntersectionListChangeCallback {
    /**
     * Callback to be invoked when the list of intersections known by Guidance changes.
     */
    void onIntersectionListChanged(List<IntersectionData> data);
}