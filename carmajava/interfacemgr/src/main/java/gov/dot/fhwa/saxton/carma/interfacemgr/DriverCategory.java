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

public enum DriverCategory {
    CONTROLLER,
    COMMS,
    POSITION,
    SENSOR,
    CAN,
    UNDEFINED;

    static DriverCategory getCat(String cat) {
        if (cat.equalsIgnoreCase("controller")) {
            return CONTROLLER;
        }else if (cat.equalsIgnoreCase("comms")) {
            return COMMS;
        }else if (cat.equalsIgnoreCase("position")) {
            return POSITION;
        }else if (cat.equalsIgnoreCase("sensor")) {
            return SENSOR;
        }else if (cat.equalsIgnoreCase("can")) {
            return CAN;
        }else {
            return UNDEFINED;
        }
    }
}
