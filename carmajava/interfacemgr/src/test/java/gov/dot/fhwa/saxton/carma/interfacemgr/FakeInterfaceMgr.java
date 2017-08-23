package gov.dot.fhwa.saxton.carma.interfacemgr;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.ArrayList;
import java.util.List;
import static org.junit.Assert.*;

public class FakeInterfaceMgr implements IInterfaceMgr {
    private Log log = LogFactory.getLog(FakeInterfaceMgr.class);

    public void bindWithDriver(String driverName){

    }


    public List<String> getDriverApi(String driverName){
        List<String> chars = new ArrayList<String>();

        if (driverName.equals("position1")) {
            chars.add("latitude");
            chars.add("longitude");

        }else if (driverName.equals("position3")) {
            chars.add("latitude");
            chars.add("longitude");
            chars.add("elevation");
            chars.add("acceleration");
        }

        return chars;
    }


    public void notifyBrokenBond(AlertSeverity sev, String message){
        String alert;

        switch(sev) {
            case fatal:     alert = "FATAL: ";    break;
            case warning:   alert = "WARN: ";     break;
            case caution:   alert = "CAUTION: ";  break;
            default:        alert = "UNDEFINED SEVERITY: ";
        }


        log.debug("notifyBrokenBond sending simulated message to system: " + alert + message);
    }
}