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

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;

/**
 * Guidance Plugin Service to abstract the finer details of planning maneuvers
 * <p>
 * Handles the abstract details of how the maneuvers interact with the controller and
 * how the maneuvers get their data from the real world.
 */
public class ManeuverPlanner {
  private IGuidanceCommands guidanceCommands;
  private IManeuverInputs maneuverInputs;

  public ManeuverPlanner(IGuidanceCommands guidanceCommands, IManeuverInputs maneuverInputs) {
    this.guidanceCommands = guidanceCommands;
    this.maneuverInputs = maneuverInputs;
  }

  /**
   * Plan a maneuver for execution in with the platform's controller and inputs.
   * <p>
   * Modifies the argument maneuver m in place
   */
  public void planManeuver(ISimpleManeuver m, double startDist) {
    m.plan(maneuverInputs, guidanceCommands, startDist);
  }

  /**
   * Plan a maneuver for execution in with the platform's controller and inputs.
   * <p>
   * Modifies the argument maneuver m in place
   */
  public double planManeuver(ISimpleManeuver m, double startDist, double endDist) {
    return m.planToTargetDistance(maneuverInputs, guidanceCommands, startDist, endDist);
  }
  
  /**
   * Check if the maneuver can be planned with vehicle lag
   */
  public boolean canPlan(ISimpleManeuver m, double startDist, double endDist) {
    return m.canPlan(maneuverInputs, startDist, endDist);
  }
  
  public IGuidanceCommands getGuidanceCommands() {
    return guidanceCommands;
  }

  public IManeuverInputs getManeuverInputs() {
    return maneuverInputs;
  }
}
