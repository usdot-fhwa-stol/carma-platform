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
 * Factory-pattern class to produce BasicAccStrategy instances
 */
public class BasicAccStrategyFactory implements IAccStrategyFactory {
  protected double vehicleResponseDelay;
  protected double desiredTimeGap;
  protected double maxAccel;

  public BasicAccStrategyFactory(double desiredTimeGap, double maxAccel, double vehicleResponseDelay) {
    this.vehicleResponseDelay = vehicleResponseDelay;
    this.desiredTimeGap = desiredTimeGap;
    this.maxAccel = maxAccel;
  }

	@Override
	public IAccStrategy newAccStrategy() {
    BasicAccStrategy strat = new BasicAccStrategy();
    strat.setDesiredTimeGap(desiredTimeGap);
    strat.setVehicleResponseDelay(vehicleResponseDelay);
    strat.setMaxAccel(maxAccel);

    return strat;
	}
}