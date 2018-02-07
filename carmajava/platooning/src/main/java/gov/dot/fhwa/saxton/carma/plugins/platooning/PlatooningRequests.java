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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

/**
 * In Mobility plan message, the first word in strategy string indicates the request type.
 * TODO This enum is not a comprehensive list. It will expand based on usage.  
 */
public enum PlatooningRequests {
    
    JOIN,
    LEAVE,
    DELEGATE,
    UPDATE;
    
    // This method returns the name of each enum without its class name 
    @Override
    public String toString() {
        return name().split(".")[1];
    }
}
