package gov.dot.fhwa.saxton.carma.guidance.plugins;

public interface AvailabilityListener {
    void onAvailabilityChange(IPlugin plugin, boolean availability);
}
