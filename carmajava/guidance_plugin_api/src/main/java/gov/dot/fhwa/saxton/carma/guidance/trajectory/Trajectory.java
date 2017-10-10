/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
  protected List<IManeuver> lateralManeuvers;
  protected List<IManeuver> longitudinalManeuvers;

  public Trajectory() {
  }

  public void addLateralManeuver(IManeuver maneuver) {
    lateralManeuvers.add(maneuver);
  }

  public void addLongitudinalManeuver(IManeuver maneuver) {
    longitudinalManeuvers.add(maneuver);
  }

  public double findEarliestWindowOfSize(double size) {
    return 0;
  }

  public double findLatestWindowOfSize(double size) {
    return 0;
  }

  public List<IManeuver> getManeuversAt(double loc) {
    return new ArrayList<IManeuver>();
  }

  public IManeuver getNextLateralManeuverAfter(double loc) {
    return null;
  }

  public IManeuver getNextLongitudinalManeuverAfter(double loc) {
    return null;
  }
}