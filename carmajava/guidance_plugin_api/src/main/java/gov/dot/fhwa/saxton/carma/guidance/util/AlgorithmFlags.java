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

package gov.dot.fhwa.saxton.carma.guidance.util;

import java.util.List;

/**
 * Data holder class for Algorithm flags contained in Route data.
 */
public class AlgorithmFlags {
  private double location;
  private List<String> disabledAlgorithms;
  
  public AlgorithmFlags(double location, List<String> disabledAlgorithms) {
    this.location = location;
    this.disabledAlgorithms = disabledAlgorithms;
  }

  /**
   * Get the location of the downtrack waypoint for the segment this set of flags pertains to, in meters
   */
  public double getLocation() {
    return location;
  }

  /**
   * Get the list of capabilities which are disabled via the route configuration for the segment this pertains to
   */
  public List<String> getDisabledAlgorithms() {
    return disabledAlgorithms;
  }
}
