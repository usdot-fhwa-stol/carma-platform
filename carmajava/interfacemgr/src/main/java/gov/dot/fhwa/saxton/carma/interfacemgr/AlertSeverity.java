package gov.dot.fhwa.saxton.carma.interfacemgr;

public enum AlertSeverity {

    CAUTION(1),
    WARNING(2),
    FATAL(3),
    NOT_READY(4),
    SYSTEM_READY(5);

    private int val_;

    AlertSeverity(int val) {
        val_ = val;
    }

    public int getVal() {
        return val_;
    }
}
