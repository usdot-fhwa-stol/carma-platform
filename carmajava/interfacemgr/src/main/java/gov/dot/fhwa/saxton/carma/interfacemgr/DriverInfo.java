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
    private boolean         can;
    private boolean         sensor;
    private boolean         position;
    private boolean         comms;
    private boolean         controller;
    private List<String>    capabilities;

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
}
