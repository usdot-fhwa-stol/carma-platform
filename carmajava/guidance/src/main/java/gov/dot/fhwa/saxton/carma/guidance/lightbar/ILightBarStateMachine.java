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

package gov.dot.fhwa.saxton.carma.guidance.lightbar;

/**
 * An interface for use within the guidance package 
 * which allows components to notify the light bar state machine of events
 */
public interface ILightBarStateMachine {
  /**
   * Function which processes events to coordinate state transitions 
   *
   * @param event the LightBarEvent which will be used to determine the next state
   */
  void next(LightBarEvent event);

  /**
   * Gets the current state
   * 
   * @return The current state of this state machine
   */
  LightBarState getCurrentState();
}