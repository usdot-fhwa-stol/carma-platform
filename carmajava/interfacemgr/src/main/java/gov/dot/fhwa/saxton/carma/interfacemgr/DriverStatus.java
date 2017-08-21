package main.java.gov.dot.fhwa.saxton.carma.interfacemgr;

public enum DriverStatus {
    // CAUTION:  Enum values must match those defined in cav_msgs/DriverStatus.msg
    off,
    operational,
    degraded,
    fault;
}
