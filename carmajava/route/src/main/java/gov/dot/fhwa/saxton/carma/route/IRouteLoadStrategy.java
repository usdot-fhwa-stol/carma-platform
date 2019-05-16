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

package gov.dot.fhwa.saxton.carma.route;

/**
 * A strategy pattern interface for use of route loading methods.
 * Implementing classes will override the load method.
 * This allows for use of multiple route loading options such route files or google maps integration.
 */
public interface IRouteLoadStrategy {
  /**
   * Loads a route into memory.
   * @return The initialized route which was loaded.
   */
  Route load();
}
