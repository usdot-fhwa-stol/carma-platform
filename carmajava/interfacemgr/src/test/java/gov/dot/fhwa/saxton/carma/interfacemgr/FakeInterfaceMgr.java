/*
 * TODO: Copyright (C) 2017 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 **/

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
        List<String> chars = new ArrayList<String>(); //must contain fully qualified name of driver + capability

        if (driverName.equals("position1")) {
            chars.add("position1/latitude");
            chars.add("position1/longitude");

        }else if (driverName.equals("position3")) {
            chars.add("position3/latitude");
            chars.add("position3/longitude");
            chars.add("position3/elevation");
            chars.add("position3/acceleration");
            
        //the first couple tests are kinda arbitrary. the next ones are formatted more like what we will find
        // in operation, with the category in the second-to-last position in the string.
        }else if (driverName.charAt(driverName.length() - 1) == '4') {
            chars.add(driverName + "/latitude");
            chars.add(driverName + "/longitude");
            chars.add(driverName + "/elevation");
        }else if (driverName.equals("position/position5")) {
            chars.add("position/position5/latitude");
            chars.add("position/position5/longitude");
            chars.add("position/position5/elevation");
        }else if (driverName.equals("radar/typeR/sensor/R5")) {
            chars.add("radar/typeR/sensor/R5/elevation");
        }

        return chars;
    }


    public void sendSystemAlert(AlertSeverity sev, String message){
        String alert;

        switch(sev) {
            case FATAL:     alert = "FATAL: ";    break;
            case WARNING:   alert = "WARN: ";     break;
            case CAUTION:   alert = "CAUTION: ";  break;
            default:        alert = "UNDEFINED SEVERITY: ";
        }


        log.debug("sendSystemAlert sending simulated message to system: " + alert + message);
    }
}