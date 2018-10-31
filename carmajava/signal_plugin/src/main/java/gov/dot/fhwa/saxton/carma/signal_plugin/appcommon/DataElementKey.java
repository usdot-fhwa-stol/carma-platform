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

package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;


public enum DataElementKey {
    TIME_SINCE_FIRST_MOTION,		// elapsed time since first motion was detected, sec
    OPERATING_SPEED,				// user-defined desired cruise speed, m/s
    SPEED_COMMAND,					// command to be sent to the XGV, m/s
    SMOOTHED_SPEED,                 // computed speed via filtering based on configured filter
    SPEED,							// actual current vehicle speed, m/s
    ACCELERATION,                   // acceleration in m/s squared
    JERK,                           // rate of acceleration  m/s -3
    LATITUDE,						// current vehicle latitude, deg
    LONGITUDE,						// current vehicle longitutde, deg
    INTERSECTION_COLLECTION,        // the collection of data describing currently known intersections
    MOTION_STATUS,                  // indicates summary of vehicle motion for DVI display
    XGV_COMMS,						// Boolean indicator whether we are communicating with XGV
    CYCLE_GPS,                      // length of consumer call() method in ms....int
    CYCLE_MAP,
    CYCLE_SPAT,
    CYCLE_XGV,
    CYCLE_EAD,
    CYCLE_XGV_COMMAND,
    XGV_STATUS,                     // XgvStatus object
    STATUS_MESSAGE,                 // StringBuffer element containing status messages
    INTERSECTION_ID,                // ID of the intersections currently being approached, 0 otherwise
    LANE_ID,                        // ID of the approach lane of the current intersections, if defined, 0 otherwise
    DIST_TO_STOP_BAR,               // DTSB on the current intersections lane, if defined, 0 otherwise
    MAP_LIST,                       // list of MAP messages captured
    SPAT_LIST,                      // list of SPAT messages captured
    SIGNAL_PHASE,
    SIGNAL_TIME_TO_NEXT_PHASE,      // seconds (double)
    SIGNAL_TIME_TO_THIRD_PHASE,     // seconds (double)
    PLANNING_START_TIME,            // seconds (double) 
    PLANNING_START_DOWNTRACK        // seconds (double)
}
