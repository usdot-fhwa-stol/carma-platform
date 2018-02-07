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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.signals.Pipeline;

/**
 * Factory-pattern class to produce BasicAccStrategy instances
 */
public class BasicAccStrategyFactory implements IAccStrategyFactory {
  protected double vehicleResponseDelay;
  protected double desiredTimeGap;
  protected double maxAccel;
  protected double minStandoffDistance;
  protected double exitDistanceFactor;
  protected Pipeline<Double> filterPipeline;
  protected static BasicAccStrategy strat;

  public BasicAccStrategyFactory(double desiredTimeGap, double maxAccel, double vehicleResponseDelay, double minStandoffDistance, double exitDistanceFactor, Pipeline<Double> filterPipeline) {
    this.vehicleResponseDelay = vehicleResponseDelay;
    this.desiredTimeGap = desiredTimeGap;
    this.maxAccel = maxAccel;
    this.minStandoffDistance = minStandoffDistance;
    this.exitDistanceFactor = exitDistanceFactor;
    this.filterPipeline = filterPipeline;
  }

  @Override
  /**
   * Note this returns a singleton instance of the ACC Strategy
   */
	public IAccStrategy newAccStrategy() {
    if (strat != null) {
      return strat;
    }
    synchronized(BasicAccStrategy.class) {
      if (strat != null) {
          return strat;
      }
      // Create ACC instance
      BasicAccStrategyFactory.strat = new BasicAccStrategy(minStandoffDistance, exitDistanceFactor, filterPipeline);
      strat.setDesiredTimeGap(desiredTimeGap);
      strat.setVehicleResponseDelay(vehicleResponseDelay);
      strat.setMaxAccel(maxAccel);
      return strat;
    }
	}
}