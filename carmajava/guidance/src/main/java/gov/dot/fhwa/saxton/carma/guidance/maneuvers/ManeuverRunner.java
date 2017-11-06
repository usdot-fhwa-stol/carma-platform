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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Runnable to handle execution of Maneuvers on regular timestep intervals
 */
public class ManeuverRunner implements Runnable {
  private long timestepDuration = 0;
  private IManeuver maneuver;

  /**
   * Construct a ManeuverRunner to invoke the IManeuver at freq Hz
   * 
   * @param maneuver The maneuver to execute. Must be already properly configured and planned.
   * @param freq The rate (in Hz) to call maneuver's execute timestep (approximate)
   */
  public ManeuverRunner(IManeuver maneuver, double freq) {
    timestepDuration = Math.round(1000 / freq);
    this.maneuver = maneuver;
  }

	@Override
	public void run() {
    long timestepStart = 0;
    long timestepEnd = 0;
    while (!Thread.currentThread().isInterrupted()) {
      timestepStart = System.currentTimeMillis();

      maneuver.executeTimeStep();

      timestepEnd = System.currentTimeMillis();

      try {
        Thread.sleep(Math.max(timestepDuration - (timestepEnd - timestepStart), 0));
      } catch (InterruptedException ie) {
        break;
      }
    }
	}
}
