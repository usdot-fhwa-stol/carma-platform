package gov.dot.fhwa.saxton.carma.guidance.params;

/**
 * Abstraction layer for org.ros.node.parameter.ParameterListener
 */
public interface ParameterListener {
    void onParameterChanged(Object param);
}
