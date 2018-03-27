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

import java.util.List;

import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;

/**
 * Interface for the component which controls the light bar
 * Plugins or GuidanceComponents can request control of indicators and set light behavior
 * The components which are granted control are determined by priorities in the configuration files
 */
public interface ILightBarManager {
  /**
   * Request control of the specified indicators
   * 
   * @param indicators The list of indicators the caller whishes to control
   * @param requestingComponent The name of the caller as specified in the configuration
   * @param lightBarChangeHandler A callback which will be triggered if the requester looses control of an indicator in the future
   * 
   * @return A list of light bar indicators which could not be controlled.
   */
  List<LightBarIndicator> requestControl(List<LightBarIndicator> indicators, String requestingComponent, ILightBarControlChangeHandler lightBarChangeHandler);

  /**
   * Set the specified indicator
   * @param indicator The indicator the caller whishes to set
   * @param status The status to set the indicator to
   * @param requestingComponent The name of the caller as specified in the configuration
   * 
   * @return True if the light could be set. False if not
   */
  boolean setIndicator(LightBarIndicator light, IndicatorStatus status, String requestingComponent);

  /**
   * Releases control of the specified indicators if the calling component has control of them
   * 
   * @param indicators The indicators to release
   * @param requestingComponent The name of the caller as specified in the configuration
   */
  void releaseControl(List<LightBarIndicator> indicators, String requestingComponent);
}