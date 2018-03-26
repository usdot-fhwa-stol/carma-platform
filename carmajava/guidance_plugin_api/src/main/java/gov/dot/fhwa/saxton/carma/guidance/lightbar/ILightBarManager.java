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
 * A ConflictDetector is responsible for providing rapid conflict detection to other guidance components
 * An internal set of vehicle paths is maintained which a suggested path can be queried against to identify conflicts
 */
public interface ILightBarManager {
  List<LightBarIndicator> requestControl(List<LightBarIndicator> indicators, String requestingComponent, ILightBarControlChangeHandler lightBarChangeHandler);

  boolean setIndicator(LightBarIndicator light, IndicatorStatus status, String requestingComponent);
}