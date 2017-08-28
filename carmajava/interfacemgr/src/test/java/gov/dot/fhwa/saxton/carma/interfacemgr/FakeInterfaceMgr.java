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
            case FATAL:     alert = "FATAL: ";    break;
            case WARNING:   alert = "WARN: ";     break;
            case CAUTION:   alert = "CAUTION: ";  break;
            default:        alert = "UNDEFINED SEVERITY: ";
        }


        log.debug("notifyBrokenBond sending simulated message to system: " + alert + message);
    }
}