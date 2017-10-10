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
import java.util.Comparator;
import java.util.List;

import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver.ManeuverType;

public class Trajectory {

  protected double startLocation;
  protected double endLocation;
  protected List<IManeuver> lateralManeuvers;
  protected List<IManeuver> longitudinalManeuvers;

  public Trajectory(double startLocation, double endLocation) {
    this.startLocation = startLocation;
    this.endLocation = endLocation;

    lateralManeuvers = new ArrayList<IManeuver>();
    longitudinalManeuvers = new ArrayList<IManeuver>();
  }

  public boolean addLateralManeuver(IManeuver maneuver) {
    if (maneuver.getType() != ManeuverType.LATERAL) {
      return false;
    }

    if (!(maneuver.getStartLocation() >= startLocation 
    && maneuver.getEndLocation() <= endLocation)) {
      return false;
    }

    return lateralManeuvers.add(maneuver);
  }

  public boolean addLongitudinalManeuver(IManeuver maneuver) {
    if (maneuver.getType() != ManeuverType.LONGITUDINAL) {
      return false;
    }

    if (!(maneuver.getStartLocation() >= startLocation 
    && maneuver.getEndLocation() <= endLocation)) {
      return false;
    }

    return longitudinalManeuvers.add(maneuver);
  }

  public double findEarliestWindowOfSize(double size) {
    if (longitudinalManeuvers.size() == 0) {
      return -1;
    }

    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });

    double lastEnd = 0;
    for (IManeuver m : longitudinalManeuvers) {
      if (m.getStartLocation() - lastEnd >= size)  {
        return lastEnd;
      }

      lastEnd = m.getEndLocation();
    }

    return -1;
  }

  public double findLatestWindowOfSize(double size) {
    if (longitudinalManeuvers.size() == 0) {
      return -1;
    }

    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });

    double lastStart = longitudinalManeuvers.get(longitudinalManeuvers.size() - 1).getEndLocation();
    for (int i = longitudinalManeuvers.size() - 1; i >= 0; i--) {
      IManeuver m = longitudinalManeuvers.get(i);
      if (lastStart - m.getEndLocation() >= size)  {
        return m.getEndLocation();
      }

      lastStart = m.getStartLocation();
    }

    return -1;
  }

  public List<IManeuver> getManeuversAt(double loc) {
    List<IManeuver> out = new ArrayList<>();

    for (IManeuver m : lateralManeuvers) {
      if (m.getStartLocation() <= loc && m.getEndLocation() > loc) {
        out.add(m);
      }
    }

    for (IManeuver m : longitudinalManeuvers) {
      if (m.getStartLocation() <= loc && m.getEndLocation() > loc) {
        out.add(m);
      }
    }

    return out;
  }

  public IManeuver getNextLateralManeuverAfter(double loc) {
    lateralManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });

    for (IManeuver m : lateralManeuvers) {
      if (m.getStartLocation() > loc) {
        return m;
      }
    }
    return null;
  }

  public IManeuver getNextLongitudinalManeuverAfter(double loc) {
    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });

    for (IManeuver m : longitudinalManeuvers) {
      if (m.getStartLocation() > loc) {
        return m;
      }
    }
    return null;
  }
}