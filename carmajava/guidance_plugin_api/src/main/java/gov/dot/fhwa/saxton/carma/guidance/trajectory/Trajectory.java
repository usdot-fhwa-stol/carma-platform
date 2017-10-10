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

/**
 * Data structure and helper method container for describing and creating vehicle trajectories
 * </p>
 * Stores both lateral and longitudinal trajectories made of maneuvers that can be individually executed
 * to command the vehicle
 */
public class Trajectory {

  protected double startLocation;
  protected double endLocation;
  protected List<IManeuver> lateralManeuvers;
  protected List<IManeuver> longitudinalManeuvers;

  /**
   * Create a new trajectory instance that will command the vehicle on distances [startLocation, endLocation)
   */
  public Trajectory(double startLocation, double endLocation) {
    this.startLocation = startLocation;
    this.endLocation = endLocation;

    lateralManeuvers = new ArrayList<IManeuver>();
    longitudinalManeuvers = new ArrayList<IManeuver>();
  }

  /**
   * Add a lateral maneuver to the Trajectory.
   * </p>
   * The maneuver will be added to the list of lateral maneuvers if the maneuver is
   * actually a lateral maneuver and falls within the space domain boundaries of the trajectory
   */
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

  /**
   * Add a longitudinal maneuver to the Trajectory.
   * </p>
   * The maneuver will be added to the list of longitudinal maneuvers if the maneuver is
   * actually a longitudinal maneuver and falls within the space domain boundaries of the trajectory
   */
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

  /**
   * Find the earliest available space in the longitudinal domain of the current trajectory for 
   * which a maneuver of the specified size might fit.
   * 
   * @returns The distance location of the start of the window if found, -1 otherwise
   */
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

  /**
   * Find the latest available space in the longitudinal domain of the current trajectory for 
   * which a maneuver of the specified size might fit.
   * 
   * @returns The distance location of the start of the window if found, -1 otherwise
   */
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

  /**
   * Get a list of all maneuvers that will be active at loc
   */
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

  /**
   * Get the next lateral maneuver which will be active after loc, null if one cannot be found
   */
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

  /**
   * Get the next longitudinal maneuver which will be active after loc, null if one cannot be found
   */
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