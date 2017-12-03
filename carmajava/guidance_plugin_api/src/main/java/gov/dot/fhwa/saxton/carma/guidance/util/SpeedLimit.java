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

/**
 * Data holder class for describing speed limit changes along a route.
 */
public class SpeedLimit {
  private double location;
  private double limit;

  public SpeedLimit(double location, double limit) {
    this.location = location;
    this.limit = limit;
  }

  /**
   * Get the downtrack distance of the end of the segment this limit pertains to, in meters.
   */
  public double getLocation() {
    return location;
  }

  /**
   * Get the upper speed limit for the segment this limit pertains to, in m/s
   */
  public double getLimit() {
    return limit;
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof SpeedLimit)) {
      return false;
    } else {
      SpeedLimit other = (SpeedLimit) o;
      return limit == other.getLimit() && location == other.getLocation();
    }
  }

  @Override
  public int hashCode() {
    return 13 * Double.hashCode(location)  + 17 * Double.hashCode(limit);
  }
}
