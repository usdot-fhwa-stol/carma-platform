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


import java.util.ArrayList;
import java.util.List;

public class DriverInfo {

    private String          name;
    private DriverState     state;
    private List<String>    capabilities;
    //the different possible categories a driver can represent. A given driver may cover multiple
    // categories, so we can't use an enum here.
    private boolean         can;
    private boolean         sensor;
    private boolean         position;
    private boolean         comms;
    private boolean         controller;

    public DriverInfo() {
        name = "";
        state = DriverState.OFF;
        can = false;
        sensor = false;
        position = false;
        comms = false;
        controller = false;
        capabilities = new ArrayList<String>();
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public DriverState getState() {
        return state;
    }

    public void setState(DriverState state) {
        this.state = state;
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

    public List<String> getCapabilities() { return capabilities; }

    public void setCapabilities(List<String> capabilities) { this.capabilities = capabilities; }

    public boolean equals(DriverInfo b) {
        if (!name.equals(b.getName())) {
            return false;
        }
        if (state != b.getState()) {
            return false;
        }
        if (can != b.isCan()) {
            return false;
        }
        if (comms != b.isComms()) {
            return false;
        }
        if (controller != b.isController()) {
            return false;
        }
        if (position != b.isPosition()) {
            return false;
        }
        if (sensor != b.isSensor()) {
            return false;
        }

        //look through all of the individual "capabilities" (api messages)
        List<String> bCapList = b.getCapabilities();
        if (capabilities.size() != bCapList.size()) {
            return false;
        }

        for (String aCap : capabilities) {
            boolean found = false;
            for (String bCap : bCapList) {
                if (aCap.equals(bCap)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                return false;
            }
        }

        return true;

    }

    /**
     * Determines if the given category is provided by the driver.
     *
     * @param cat - category in question (may be UNDEFINED, in which case a driver always matches)
     * @return true if the driver does fall into the given category
     */
    protected boolean hasCategory(DriverCategory cat) {

        if (cat == DriverCategory.UNDEFINED) {
            return true;
        }

        if ((cat == DriverCategory.CONTROLLER   &&  controller)   ||
                (cat == DriverCategory.COMMS    &&  comms)        ||
                (cat == DriverCategory.CAN      &&  can)          ||
                (cat == DriverCategory.POSITION &&  position)     ||
                (cat == DriverCategory.SENSOR   &&  sensor)) {
            return true;
        }

        return false;
    }
}
