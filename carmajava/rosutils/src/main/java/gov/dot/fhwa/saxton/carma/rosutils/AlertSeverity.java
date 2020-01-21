/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.rosutils;

public enum AlertSeverity {

    CAUTION(1),
    WARNING(2),
    FATAL(3),
    NOT_READY(4),
    DRIVERS_READY(5),
    SHUTDOWN(6);

    private int val_;

    AlertSeverity(int val) {
        val_ = val;
    }

    public int getVal() {
        return val_;
    }
}
