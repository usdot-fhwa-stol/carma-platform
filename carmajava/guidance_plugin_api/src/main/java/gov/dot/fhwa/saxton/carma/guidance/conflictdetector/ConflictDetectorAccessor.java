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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

/**
 * Static class for accessing a shared instance of a IConflictDetector
 * During regular CARMA execution,
*  the setConflictDetector function will be called in the guidance package on initialize
 */
public final class ConflictDetectorAccessor {

  private static IConflictDetector conflictDetector;

  /**
   * Sets the conflict detector which will be shared by users of this class
   * Note: During CARMA execution this function should only be called in the guidance package
   * 
   * @param conflictDetector The conflict detector to set
   */
  public static void setConflictDetector(IConflictDetector conflictDetector) {
    ConflictDetectorAccessor.conflictDetector = conflictDetector;
  }
  
  /**
   * Get the shared conflict detector instance
   * 
   * @return an instance of a IConflictDetector or null if setConflictDetector has not been called in any thread
   */
  public static IConflictDetector getConflictDetector() {
    return conflictDetector;
  }
}