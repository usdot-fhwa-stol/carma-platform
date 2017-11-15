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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Static manager class that holds the selected ACC strategy factory and provides
 * access to it for maneuvers.
 */
public class AccStrategyManager {
  protected static IAccStrategyFactory accFactory;

  /**
   * Set the ACC strategy factory to be used by all new maneuvers.
   * <p>
   * Should only be called in GuidanceMain based on configuration from param files
   */
  public static void setAccStrategyFactory(IAccStrategyFactory factory) {
    accFactory = factory;
  }

  /**
   * Get a new instance of the currently configured IAccStrategy by invoking the factory
   */
  public static IAccStrategy newAccStrategy() {
    return accFactory.newAccStrategy();
  }
}