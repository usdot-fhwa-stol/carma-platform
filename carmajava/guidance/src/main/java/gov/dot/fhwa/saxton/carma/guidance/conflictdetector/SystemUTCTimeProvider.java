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

package gov.dot.fhwa.saxton.carma.guidance.conflictdetector;

/**
 * Simple interface for determining the current time in seconds or milliseconds
 * The time provided is relative to the UTC epoch
 * The epoch began Jan 1, 1970 00:00:00 UTC
 * 
 * Time is obtained from the unix system clock
 */
public class SystemUTCTimeProvider implements IMobilityTimeProvider {
  
  @Override
  public double getCurrentTimeSeconds() {
    return System.currentTimeMillis() / 1000L;
  }

  @Override
  public long getCurrentTimeMillis() {
    return System.currentTimeMillis();
  }
}