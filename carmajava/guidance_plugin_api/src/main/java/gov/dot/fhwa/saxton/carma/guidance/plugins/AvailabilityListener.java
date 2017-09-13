package gov.dot.fhwa.saxton.carma.guidance.plugins;

/**
 * Event listener for being notified when an IPlugin instance changes availability state
 */
public interface AvailabilityListener {
    void onAvailabilityChange(IPlugin plugin, boolean availability);
}
