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
  protected boolean lateralManeuversSorted = true;
  protected boolean longitudinalManeuversSorted = true;

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
   * Get the location along the route that this Trajectory will start at
   */
  public double getStartLocation() {
    return this.startLocation;
  }

  /**
   * Get the location along the route that this Trajectory will finish at
   */
  public double getEndLocation() {
    return this.startLocation;
  }

  /**
   * Add a maneuver to the Trajectory.
   * </p>
   * The maneuver will be added to the appropriate maneuvers list if it fits spatially within the domain of
   * this trajectory instance.
   */
  public boolean addManeuver(IManeuver maneuver) {
    if (!(maneuver.getStartLocation() >= startLocation 
    && maneuver.getEndLocation() <= endLocation)) {
      return false;
    }

    if (maneuver.getType() == ManeuverType.LATERAL) {
      lateralManeuversSorted = false;
      return lateralManeuvers.add(maneuver);
    }

    if (maneuver.getType() == ManeuverType.LONGITUDINAL) {
      longitudinalManeuversSorted = false;
      return longitudinalManeuvers.add(maneuver);
    }

    return false;
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

    sortLongitudinalManeuvers();

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

  private void sortLateralManeuvers() {
    lateralManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });
  }

  private void sortLongitudinalManeuvers() {
    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
		@Override
		public int compare(IManeuver o1, IManeuver o2) {
			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
		}
    });
  }

  /**
   * Get the next maneuver of the specified type which will be active after loc, null if one cannot be found
   */
  public IManeuver getNextManeuverAfter(double loc, ManeuverType type) {
    if (type == ManeuverType.LONGITUDINAL) {
      sortLongitudinalManeuvers();

      for (IManeuver m : longitudinalManeuvers) {
        if (m.getStartLocation() > loc) {
          return m;
        }
      }

      return null;
    } 

    if (type == ManeuverType.LATERAL) {
      sortLateralManeuvers();

      for (IManeuver m : lateralManeuvers) {
        if (m.getStartLocation() > loc) {
          return m;
        }
      }

      return null;
    }

    return null;
  }

  /**
   * Get the trajectories stored lateral maneuvers in sorted order by start location
   */
  public List<IManeuver> getLateralManeuvers() {
    sortLateralManeuvers();
    return this.lateralManeuvers;
  }

  /**
   * Get the trajectories stored longitudinal maneuvers in sorted order by start location
   */
  public List<IManeuver> getLongitudinalManeuvers() {
    sortLongitudinalManeuvers();
    return this.longitudinalManeuvers;
  }

  /**
   * Get the trajectories stored maneuvers in sorted order by start location
   */
  public List<IManeuver> getManeuvers() {
    List<IManeuver> out = new ArrayList<IManeuver>();
    out.addAll(longitudinalManeuvers);
    out.addAll(lateralManeuvers);

    out.sort(new Comparator<IManeuver>() {
        @Override
        public int compare(IManeuver o1, IManeuver o2) {
          return Double.compare(o1.getStartLocation(), o2.getStartLocation());
        }
      });

    return out;
  }
}
