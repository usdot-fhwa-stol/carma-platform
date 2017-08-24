package gov.dot.fhwa.saxton.carma.interfacemgr;

public enum DriverState {
    // CAUTION:  Enum values must match those defined in cav_msgs/DriverState.msg
    off,
    operational,
    degraded,
    fault;
}
