package gov.dot.fhwa.saxton.carma.interfacemgr;


public class DriverInfo {

    private String          name;
    private DriverState     status;
    private boolean         can;
    private boolean         sensor;
    private boolean         position;
    private boolean         comms;
    private boolean         controller;
    private int             id;

    public DriverInfo() {
        name = "";
        status = DriverState.off;
        can = false;
        sensor = false;
        position = false;
        comms = false;
        controller = false;
        id = 0;
    }


    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public DriverState getStatus() {
        return status;
    }

    public void setStatus(DriverState status) {
        this.status = status;
    }

    public boolean isCan() {
        return can;
    }

    public void setCan(boolean can) {
        this.can = can;
    }

    public boolean isSensor() {
        return sensor;
    }

    public void setSensor(boolean sensor) {
        this.sensor = sensor;
    }

    public boolean isPosition() {
        return position;
    }

    public void setPosition(boolean position) {
        this.position = position;
    }

    public boolean isComms() {
        return comms;
    }

    public void setComms(boolean comms) {
        this.comms = comms;
    }

    public boolean isController() {
        return controller;
    }

    public void setController(boolean controller) {
        this.controller = controller;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
