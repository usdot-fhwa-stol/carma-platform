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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.utils.ComponentVersion;

import java.io.InputStream;
import java.util.Scanner;

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
        int intermediate =  8;
        int minor =         4;
        // Don't touch this, automatically updated:
        int build = 0;
        String suffix = "";

        ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
        InputStream versionFileStream = classLoader.getResourceAsStream("version");

        try (Scanner scanner = new Scanner(versionFileStream)) {
            if (scanner.hasNextLine()) {
                try {
                    build = Integer.parseInt(scanner.nextLine());
                } catch (NumberFormatException nfe) {
                    build = 0;
                }
            } else {
                build = 0;
            }

            if (scanner.hasNextLine()) {
                suffix = scanner.nextLine();
            } else {
                suffix = "VERSION-FILE-ERROR";
            }
        } 


//==============================================================================================================================


        version.setName(name);
        version.setMajorRevision(major);

        //if any one of the below items is not explicitly set it will not be displayed in the revision string.
        version.setIntermediateRevision(intermediate);
        version.setMinorRevision(minor);
        version.setBuild(build);
        version.setSuffix(suffix);

        return version;
    }
}
