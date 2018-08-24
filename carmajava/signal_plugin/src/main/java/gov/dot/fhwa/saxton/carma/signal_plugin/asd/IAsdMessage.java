/*
 * Copyright (C) 2018 LEIDOS.
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
 * Interface for a concrete ASD message
 */
public interface IAsdMessage {

    /**
     * Interface method to convert the provided packet byte array into an ASD specific message
     *
     * @param buf
     * @return true if the message was successfully parsed
     */
    public boolean parse(byte[] buf);


    /**
     * Returns the ID of the intersections that this message applies to.
     * @return intersections ID
     */
    int getIntersectionId();


    /**
     * Returns the version number of the message.
     * @return version number
     */
    int getContentVersion();
}
