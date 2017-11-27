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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IComplexManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ISimpleManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * Data structure and helper method container for describing and creating vehicle trajectories
 * </p>
 * Stores both lateral and longitudinal trajectories made of maneuvers that can be individually executed
 * to command the vehicle
 */
public class Trajectory {

  protected double startLocation;
  protected double endLocation;
  protected Set<ISimpleManeuver> lateralManeuvers;
  protected Set<LongitudinalManeuver> longitudinalManeuvers;
  protected IComplexManeuver complexManeuver = null;
  protected boolean lateralManeuversSorted = true;
  protected boolean longitudinalManeuversSorted = true;

  /**
   * Create a new trajectory instance that will command the vehicle on distances [startLocation, endLocation)
   */
  public Trajectory(double startLocation, double endLocation) {
    this.startLocation = startLocation;
    this.endLocation = endLocation;

    Comparator<IManeuver> sortingComparator = (IManeuver a, IManeuver b) -> {
      return Double.compare(a.getStartDistance(), b.getStartDistance());
    };

    lateralManeuvers = new TreeSet<>(sortingComparator);
    longitudinalManeuvers = new TreeSet<>(sortingComparator);
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
    return this.endLocation;
  }

  /**
   * Compute if the ranges [start1, end1) and [start2, end2) overlap by analyizing the boundary points
   */
	private boolean checkOverlap(double start1, double end1, double start2, double end2) {
		// Start2 is inside the first range
		if (start2 >= start1 && start2 < end1) {
			return true;
		}
		//End2 is inside the range
		if (end2 > start1 && end2 < end1) { 
			return true;
		}
		// The whole of range 2 overlaps range1
		if (start2 <= start1 && end2 >= end1) {
			return true;
		}

		// Start2 is inside the first range
		if (start1 >= start2 && start1 < end2) {
			return true;
		}
		//End2 is inside the range
		if (end1 > start2 && end1 < end2) { 
			return true;
		}
		// The whole of range 2 overlaps range1
		if (start1 <= start2 && end1 >= end2) {
			return true;
		}

		return false;
	}

  /**
   * Add a maneuver to the Trajectory.
   * </p>
   * The maneuver will be added to the appropriate maneuvers list if it fits spatially within the domain of
   * this trajectory instance.
   */
  public boolean addManeuver(ISimpleManeuver maneuver) {
    if (!(maneuver.getStartDistance() >= startLocation 
    && maneuver.getEndDistance() <= endLocation)) {
      return false;
    }

    // Not a fan of using instanceof here, but without more information exposed by the maneuvers, not much I can do
    if (maneuver instanceof LongitudinalManeuver) {

      // Check for overlap of maneuvers
      for (ISimpleManeuver m : getLongitudinalManeuvers()) {
        if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), m.getStartDistance(), m.getEndDistance())) {
          return false;
        }
      }

      IComplexManeuver cm = getComplexManeuver();
      if (cm != null) {
        if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), cm.getStartDistance(), cm.getEndDistance())) {
          return false;
        }
      }

      return longitudinalManeuvers.add((LongitudinalManeuver) maneuver);
    } else {

      // Check for overlap of maneuvers
      for (ISimpleManeuver m : getLateralManeuvers()) {
        if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), m.getStartDistance(), m.getEndDistance())) {
          return false;
        }
      }

      IComplexManeuver cm = getComplexManeuver();
      if (cm != null) {
        if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), cm.getStartDistance(), cm.getEndDistance())) {
          return false;
        }
      }

      return lateralManeuvers.add(maneuver);
    }
  }

  /**
   * Set the complex maneuver for the current trajectory.
   * <p>
   * In order to accept a complex maneuver, there must not already be a complex maneuver for this trajectory,
   * the requested complex maneuver must be within trajectory boundaries, and the requested complex maneuver
   * must be the last maneuver in the trajectory. If no maneuvers are presently planned after the start of
   * the complex maneuver, the end of this trajectory will be set to the end of the complex manauever.
   * 
   * @param maneuver The complex maneuver to to add
   * @return True if the maneuver has been accepted, false o.w.
   */
  public boolean setComplexManeuver(IComplexManeuver maneuver) {
    if (maneuver == null) {
      return false;
    }

    if (complexManeuver != null) {
      // Only one complex maneuver is allowed per trajectory
      return false;
    }

    if (!(maneuver.getStartDistance() >= startLocation && maneuver.getEndDistance() <= endLocation)) {
      // Must be within bounds like normal maneuvers
      return false;
    }

    if (getNextManeuverAfter(maneuver.getStartDistance(), ManeuverType.LONGITUDINAL) != null ||
        getNextManeuverAfter(maneuver.getStartDistance(), ManeuverType.LATERAL) != null) {
      // Complex maneuver must be last maneuver in trajectory
      return false;
    }

    // Check for overlap with any other maneuvers currentl planned
    for (IManeuver m : lateralManeuvers) {
      if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), m.getStartDistance(), m.getEndDistance())) {
        return false;
      }
    }

    for (IManeuver m : longitudinalManeuvers) {
      if (checkOverlap(maneuver.getStartDistance(), maneuver.getEndDistance(), m.getStartDistance(), m.getEndDistance())) {
        return false;
      }
    }

    // Valid complex maneuver received, adjust and accept
    endLocation = maneuver.getEndDistance();
    complexManeuver = maneuver;
    return true;
  }

  /**
   * Get the complex maneuver contained in this trajectory, if it exists.
   * 
   * @return The planned {@link IComplexManeuver} instance if it exists, null o.w.
   */
  public IComplexManeuver getComplexManeuver() {
    return complexManeuver;
  }

  /**
   * Find the earliest available space in the longitudinal domain of the current trajectory for 
   * which a maneuver of the specified size might fit.
   * 
   * @returns The distance location of the start of the window if found, -1 otherwise
   */
  public double findEarliestWindowOfSize(double size) {
    List<IManeuver> maneuvers = new ArrayList<>();
    maneuvers.addAll(longitudinalManeuvers);
    if (complexManeuver != null) {
      maneuvers.add(complexManeuver);
    }

    if (maneuvers.size() == 0) {
      return -1;
    }

    maneuvers.sort((IManeuver m1, IManeuver m2) -> 
      Double.compare(m1.getStartDistance(), m2.getStartDistance()));

    double lastEnd = 0;
    for (IManeuver m : maneuvers) {
      if (m.getStartDistance() - lastEnd >= size)  {
        return lastEnd;
      }

      lastEnd = m.getEndDistance();
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
    List<IManeuver> maneuvers = new ArrayList<>();
    maneuvers.addAll(longitudinalManeuvers);
    if (complexManeuver != null) {
      maneuvers.add(complexManeuver);
    }

    if (maneuvers.size() == 0) {
      return -1;
    }

    maneuvers.sort((IManeuver m1, IManeuver m2) -> {
      return Double.compare(m1.getStartDistance(),
        m2.getStartDistance());
    });

    double lastStart = maneuvers.get(maneuvers.size() - 1).getEndDistance();
    for (int i = maneuvers.size() - 1; i >= 0; i--) {
      IManeuver m = maneuvers.get(i);
      if (lastStart - m.getEndDistance() >= size)  {
        return m.getEndDistance();
      }

      lastStart = m.getStartDistance();
    }

    return -1;
  }

  /**
   * Get a list of all maneuvers that will be active at loc
   */
  public List<IManeuver> getManeuversAt(double loc) {
    List<IManeuver> out = new ArrayList<>();

    for (ISimpleManeuver m : lateralManeuvers) {
      if (m.getStartDistance() <= loc && m.getEndDistance() > loc) {
        out.add(m);
      }
    }

    for (ISimpleManeuver m : longitudinalManeuvers) {
      if (m.getStartDistance() <= loc && m.getEndDistance() > loc) {
        out.add(m);
      }
    }

    if (complexManeuver != null) {
      if (complexManeuver.getStartDistance() <= loc && complexManeuver.getEndDistance() > loc) {
        out.add(complexManeuver);
      }
    }

    return out;
  }

  /**
   * Get a list of all maneuver of a specific type that will be active at loc
   * Undefined behavior if there are overlapping maneuvers of the same type
   */
  public IManeuver getManeuverAt(double loc, ManeuverType type) {
    if (type == ManeuverType.LATERAL) {
      for (ISimpleManeuver m : lateralManeuvers) {
        if (m.getStartDistance() <= loc && m.getEndDistance() > loc) {
          return m;
        }
      }
    }

    if (type == ManeuverType.LONGITUDINAL) {
      for (ISimpleManeuver m : longitudinalManeuvers) {
        if (m.getStartDistance() <= loc && m.getEndDistance() > loc) {
          return m;
        }
      }
    }

    if (type == ManeuverType.COMPLEX) {
      if (complexManeuver != null) {
        if (complexManeuver.getStartDistance() <= loc && complexManeuver.getEndDistance() > loc) {
          return complexManeuver;
        }
      }
    }

    return null;
  }
  /**
   * Get the next maneuver of the specified type which will be active after loc, null if one cannot be found
   */
  public IManeuver getNextManeuverAfter(double loc, ManeuverType type) {
    if (type == ManeuverType.LONGITUDINAL) {
      for (ISimpleManeuver m : longitudinalManeuvers) {
        if (m.getStartDistance() > loc) {
          return m;
        }
      }

      return null;
    } 

    if (type == ManeuverType.LATERAL) {
      for (ISimpleManeuver m : lateralManeuvers) {
        if (m.getStartDistance() > loc) {
          return m;
        }
      }

      return null;
    }

    if (type == ManeuverType.COMPLEX) {
      if (complexManeuver != null) {
        if (loc < complexManeuver.getStartDistance()) {
          return complexManeuver;
        }
      }

      return null;
    }

    return null;
  }

  /**
   * Get the trajectories stored lateral maneuvers in sorted order by start location
   */
  public List<ISimpleManeuver> getLateralManeuvers() {
    return new ArrayList<>(lateralManeuvers);
  }

  /**
   * Get the trajectories stored longitudinal maneuvers in sorted order by start location
   */
  public List<LongitudinalManeuver> getLongitudinalManeuvers() {
    return new ArrayList<>(longitudinalManeuvers);
  }

  /**
   * Get the trajectories stored maneuvers in sorted order by start location
   */
  public List<IManeuver> getManeuvers() {
    List<IManeuver> out = new ArrayList<>();
    out.addAll(longitudinalManeuvers);
    out.addAll(lateralManeuvers);
    
    if (complexManeuver != null) {
      out.add(complexManeuver);
    }

    // I can't think of a notable performance different between making a new TreeSet vs making this list and sorting it
    // If anything the list is the output format, so it's O(n) better since we don't have to copy the set into the list
    out.sort((IManeuver o1, IManeuver o2) -> Double.compare(o1.getStartDistance(), o2.getStartDistance()));

    return out;
  }
}
