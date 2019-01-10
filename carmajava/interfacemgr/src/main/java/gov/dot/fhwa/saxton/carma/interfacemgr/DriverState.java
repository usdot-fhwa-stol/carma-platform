/*
 * Copyright (C) 2018-2019 LEIDOS.
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
 */

package gov.dot.fhwa.saxton.carma.interfacemgr;

public enum DriverState {
    // CAUTION:  Enum values must match those defined in cav_msgs/DriverState.msg
    OFF(0),
    OPERATIONAL(1),
    DEGRADED(2),
    FAULT(3);

    private int val_;

    DriverState(int val) {
        val_ = val;
    }
}
