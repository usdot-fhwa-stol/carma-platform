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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;

/**
 * Factory-pattern class for ICostModel objects
 * <p>
 * Used by the Signal Plugin to determine which cost model is being used for the A-Star search
 */
public interface ICostModelFactory {

  /**
   * Create a new instance of ICostModel
   * 
   * @param desiredModelName A string identifying the cost model which will be used
   * 
   * @throws IllegalArgumentException Exception thrown if the desired cost model could not be instantiated
   * 
   * @return An initialized ICostModel object
   */
  public ICostModel getCostModel(String desiredModelName) throws IllegalArgumentException ;
}