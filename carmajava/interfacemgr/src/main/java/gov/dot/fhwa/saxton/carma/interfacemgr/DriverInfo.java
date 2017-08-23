package gov.dot.fhwa.saxton.carma.interfacemgr;


public class DriverInfo {

    private String          name;
    private DriverStatus    status;
    private boolean         can;
    private boolean         sensor;
    private boolean         position;
    private boolean         comms;
    private boolean         controller;
    private int             id;


    //TODO - generate remaining accessors

    public boolean isComms() {
        return comms;
    }

    public void setComms(boolean comms) {
        this.comms = comms;
    }

}
