/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

/**
 * Static class for accessing a shared instance of a ITrajectoryConverter
 * During regular CARMA execution,
*  the setTrajectoryConverter function will be called in the guidance package on initialize
 */
public final class TrajectoryConverterAccessor {

  private static ITrajectoryConverter trajectoryConverter;

  /**
   * Sets the trajectory converter which will be shared by users of this class
   * Note: During CARMA execution this function should only be called in the guidance package
   * 
   * @param trajectoryConverter The trajectory converter to set
   */
  public static void setTrajectoryConverter(ITrajectoryConverter trajectoryConverter) {
    TrajectoryConverterAccessor.trajectoryConverter = trajectoryConverter;
  }
  
  /**
   * Get the shared trajectory converter instance
   * 
   * @return an instance of a ITrajectoryConverter or null if setTrajectoryConverter has not been called in any thread
   */
  public static ITrajectoryConverter getTrajectoryConverter() {
    return trajectoryConverter;
  }
}