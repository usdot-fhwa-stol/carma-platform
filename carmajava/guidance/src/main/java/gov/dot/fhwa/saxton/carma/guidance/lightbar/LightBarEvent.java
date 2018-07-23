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
 * Events which can trigger a light bar state change
 */
public enum LightBarEvent {
  /**
   * Guidance is disengaged
   */
  GUIDANCE_DISENGAGED,
  /**
   * Guidance is engaged
   */
  GUIDANCE_ENGAGED,
  /**
   * DSRC messages are being received
   */
  DSRC_MESSAGE_RECEIVED,
  /**
   * No DSRC messages have been received recently
   */
  DSRC_MESSAGE_TIMEOUT;
}