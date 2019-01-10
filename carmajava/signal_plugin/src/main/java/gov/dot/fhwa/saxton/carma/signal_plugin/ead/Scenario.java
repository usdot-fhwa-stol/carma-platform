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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

public enum Scenario {
	RAMP_UP,		//initial acceleration to operating speed at beginning of test run
	CONSTANT,		//no speed change required (scenario 1 in white paper)
	OVERSPEED,		//speed up to get through intersections before green expires (scenario 2)
	OVERSPEED_EXT,	//continuation of the overspeed state while vehicle passes through the stop box (scenario 2)
	SLOWING,		//slow down to wait for red to expire before we reach stop bar (scenario 4)
	GRADUAL_STOP,	//moving toward an eventual stop at stop bar and wait for red to expire (scenario 3)
	FINAL_STOP,		//final moments of the stopping maneuver (scenario 3)
	DEPARTURE		//egress from the intersections after crossing the stop bar
}
