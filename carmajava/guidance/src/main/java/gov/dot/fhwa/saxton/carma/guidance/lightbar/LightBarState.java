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

/**
 * The state of the state machine used to control the light bar from within guidance. 
 */
public enum LightBarState {
  /**
   * Guidance is disengage, the light bar should be off
   */
  DISENGAGED,
  /**
   * Guidance is engaged, the light bar should show solid green
   */
  ENGAGED,
  /**
   * Guidance is receiving DSRC messages, the light bar should be blinking green
   */
  RECEIVING_MESSAGES,
  /**
   * A negotiation is in progress, the light bar should be blinking yellow
   */
  NEGOTIATING;
}