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

import java.util.List;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Interface for a hashing strategy used to generate cell coordinates for an NSpacialHashMap
 */
public interface IConflictManager extends IConflictDetector{
  // Maybe add and remove should be boolean?
  void addPath(cav_msgs.MobilityPath pathMsg);
  void removePath(cav_msgs.MobilityPath pathMsg); // TODO might be better to just have vehicle id passed in
}