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

package gov.dot.fhwa.saxton.carma.rsumetering;

/**
 * Struct for storing data about a platoon 
 */
public final class PlatoonData {
  protected final String leaderId; 
  protected final double rearDTD; // m
  protected final double speed; // m/s
  protected final long expectedTimeOfArrival; // ms
  protected final String rearBSMId; 
  protected final long stamp;
  protected final static double LOC_EPSILON_M = 0.1; // m
  protected final static double SPEED_EPSILON_MPS = 0.1; // m /s

  /**
   * Constructor
   * 
   * @param leaderId The static id of the platoon leader
   * @param rearDTD The downtrack distance of the back of the platoon in m
   * @param speed The speed of the platoon in m/s
   * @param expectedTimeOfArrival The expected UTC time of arrival at the merge point in ms
   * @param rearBSMId The BSM id of the rear vehicle in this platoon
   * @param stamp The time stamp of this platoon data
   */
  public PlatoonData (String leaderId, double rearDTD, 
    double speed, long expectedTimeOfArrival, String rearBSMId, long stamp) {
    this.leaderId = leaderId;
    this.rearDTD = rearDTD;
    this.speed = speed;
    this.expectedTimeOfArrival = expectedTimeOfArrival;
    this.rearBSMId = rearBSMId;
    this.stamp = stamp;
  }
  
  /**
   * @return the leaderId
   */
  public String getLeaderId() {
    return leaderId;
  }

  /**
   * @return the rearDTD
   */
  public double getRearDTD() {
    return rearDTD;
  }

  /**
   * @return the speed
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * @return the expectedTimeOfArrival
   */
  public long getExpectedTimeOfArrival() {
    return expectedTimeOfArrival;
  }

  /**
   * @return the rearBSMId
   */
  public String getRearBSMId() {
    return rearBSMId;
  }
  
  /**
   * @return the stamp
   */
  public long getStamp() {
    return stamp;
  }

  /**
   * Comparison of the objects' floating point values is done with an epsilon
   * See RampMeterData.LOC_EPSILON_M for this value
   */
  @Override
  public boolean equals(Object o) {
      if (this == o)
          return true;
      if (o == null || !(o instanceof PlatoonData))
          return false;
        
          PlatoonData data = (PlatoonData) o;

      return leaderId.equals(data.getLeaderId()) 
        && Math.abs(data.getRearDTD() - rearDTD) < LOC_EPSILON_M
        && Math.abs(data.getSpeed() - speed) < SPEED_EPSILON_MPS
        && data.getExpectedTimeOfArrival() == expectedTimeOfArrival;
  }

  @Override
  public String toString() {
    final String fields = "PlatoonData{ leaderId: %s, rearDTD: %.2f, speed: %.2f, timeOfArrival: %d, rearBSMId: %s, msgStamp: %d}";
    return String.format(fields, leaderId, rearDTD, speed, expectedTimeOfArrival, rearBSMId, stamp);
  }
}