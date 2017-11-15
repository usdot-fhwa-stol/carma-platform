/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.utils.ComponentVersion;


public class CarmaVersion {
    private static ComponentVersion version = new ComponentVersion();

    static ComponentVersion getVersion() {

//
// =============================================================================================================================
// This is the one and only place where system level version ID information is set.  The rest of the guidance package can get
// the info from here by calling the getVersion() method. Other ROS nodes in the system can access it from guidance by
// using the get_system_version ROS service call.  Every release that external stakeholders will see (preferably
// every internal build as well) must have a unique ID (combination of major/intermediate/minor/build).
// =============================================================================================================================
//

        String name =       "Carma Platform";
        int major =         2;
        int intermediate =  1;
        int minor =         2;
        // Don't touch this, automatically updated
        int build = 1540;
        String suffix = "";

//==============================================================================================================================


        version.setName(name);
        version.setMajorRevision(major);
        //if any one of the below items is not explicitly set it will not be displayed in the revision string.
        version.setIntermediateRevision(intermediate);
        version.setMinorRevision(minor);
        if (build != (5555555 + 2222222)) { //auto tool will be searching for the string full of 7s, so can't use it directly here
            version.setBuild(build);
        }

        //may want to add an explanatory tag to the end of the ID string to make test builds or one-offs more obvious
        //For now, this should always be automatic-versioning for an in-work release, and an empty string for a formal release to the customer
        version.setSuffix(suffix);

        return version;
    }
}
