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

package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

/**
 * Struct for storing data about a RSU Ramp Metering infrastructure component
 */
public final class RampMeterData {
  protected final String rsuId; 
  protected final double rampMeterDTD; // m
  protected final double mergePointDTD; // m
  protected final double mergeLength; // m
  protected final double radius; // m
  protected final static double LOC_EPSILON_M = 0.1; // m

  /**
   * Constructor
   * 
   * @param rsuId The static id of the rsu
   * @param rampMeterDTD The downtrack distance of the rsu
   * @param mergePointDTD The downtrack distance of the merge point
   * @param mergeLength The length of the merge region
   * @param radius The radius around the merge point where a vehicle may request a merge
   */
  public RampMeterData (String rsuId, double rampMeterDTD, 
    double mergePointDTD, double mergeLength, double radius) {
    this.rsuId = rsuId;
    this.rampMeterDTD = rampMeterDTD;
    this.mergePointDTD = mergePointDTD;
    this.mergeLength = mergeLength;
    this.radius = radius;
  }
  /**
   * @return the rsuId
   */
  public String getRsuId() {
    return rsuId;
  }

  /**
   * @return the rampMeterDTD
   */
  public double getRampMeterDTD() {
    return rampMeterDTD;
  }

  /**
   * @return the mergePointDTD
   */
  public double getMergePointDTD() {
    return mergePointDTD;
  }

  /**
   * @return the mergeLength
   */
  public double getMergeLength() {
    return mergeLength;
  }

  /**
   * @return the radius
   */
  public double getRadius() {
    return radius;
  }

  /**
   * Comparison of the objects' floating point values is done with an epsilon
   * See RampMeterData.LOC_EPSILON_M for this value
   */
  @Override
  public boolean equals(Object o) {
      if (this == o)
          return true;
      if (o == null || !(o instanceof RampMeterData))
          return false;
        
      RampMeterData data = (RampMeterData) o;

      return rsuId.equals(data.getRsuId()) 
        && Math.abs(data.getRampMeterDTD() - rampMeterDTD) < LOC_EPSILON_M
        && Math.abs(data.getMergePointDTD() - mergePointDTD) < LOC_EPSILON_M
        && Math.abs(data.getMergeLength() - mergeLength) < LOC_EPSILON_M
        && Math.abs(data.getRadius() - radius) < LOC_EPSILON_M;
  }
}