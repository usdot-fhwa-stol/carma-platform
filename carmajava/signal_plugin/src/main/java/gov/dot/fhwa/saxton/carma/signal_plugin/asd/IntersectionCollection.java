package gov.dot.fhwa.saxton.carma.signal_plugin.asd;

import java.util.List;

/**
 * Collection of all known intersections at the current time.
 */
public class IntersectionCollection {
    public List<IntersectionData> intersections; //ordered from nearest (element 0) to farthest from vehicle
}
