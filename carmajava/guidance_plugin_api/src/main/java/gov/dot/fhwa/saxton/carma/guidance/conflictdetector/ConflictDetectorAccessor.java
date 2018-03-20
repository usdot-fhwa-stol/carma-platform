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
 * Interface for a ConflictManager
 * A ConflictManager extends the behavior of a ConflictDetector 
 * by providing an interface for adding or removing current vehicle paths to the conflict detection system
 */
public final class ConflictDetectorAccessor {

  private static IConflictDetector conflictDetector;

  public static void setConflictDetector(IConflictDetector conflictDetector) {
    ConflictDetectorAccessor.conflictDetector = conflictDetector;
  }
  
  public static IConflictDetector getConflictDetector() {
    return conflictDetector;
  }
}