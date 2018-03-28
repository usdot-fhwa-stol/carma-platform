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

package gov.dot.fhwa.saxton.carma.guidance.lightbar;

import java.util.Arrays;
import java.util.List;

/**
 * Enum of possible light bar indicators which can be controlled
 */
public enum LightBarIndicator {
  /**
   * Center green light
   */
  GREEN,
  /**
   * Side yellow lights
   */
  YELLOW;

  private static final List<LightBarIndicator> ALL_INDICATORS = 
    Arrays.asList(
      GREEN,
      YELLOW
    );

  /**
   * Returns a list of all the indicators
   * 
   * @return list of each unique indicator
   */
  public static final List<LightBarIndicator> getListOfAllIndicators() {
    return ALL_INDICATORS;
  }
}