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

package gov.dot.fhwa.saxton.carma.signal_plugin.asd;

/**
 * Enum defining the type of Asd Message (MAP or SPAT)
 *
 * The type is the first byte within the raw packet that defines the raw data associated with the message type
 *
 * User: ferenced
 * Date: 1/19/15
 * Time: 9:24 AM
 *
 */
public enum AsdMessageType {
    MAP_MSG_ID (0x87),
    SPAT_MSG_ID (0x8D);

    private int type;
    AsdMessageType(int i) {
        this.type = i;
    }

    public int getType() {
        return this.type;
    }
}

