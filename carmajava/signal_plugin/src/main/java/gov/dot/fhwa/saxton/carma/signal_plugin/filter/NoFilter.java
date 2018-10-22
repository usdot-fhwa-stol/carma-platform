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

package gov.dot.fhwa.saxton.carma.signal_plugin.filter;

import gov.dot.fhwa.saxton.carma.signal_plugin.filter.IDataFilter;

/**
 * This class is a simple substitution for accessing the raw speed data stream directly, 
 * but allows the architecture to use the filtering approach at all times.
 * 
 * @author starkj
 *
 */
public class NoFilter implements IDataFilter {
	
	@Override
	public void initialize(double timeStep) {
		current_ = 0.0;
		prev1_ = 0.0;
		prev2_ = 0.0;
		timeStep_ = timeStep;
	}

	@Override
	public void addRawDataPoint(double rawValue) {
		//need to store current point plus two historical points
		prev2_ = prev1_;
		prev1_ = current_;
		current_ = rawValue;
	}

	@Override
	public double getSmoothedValue() {
		return current_;
	}

	@Override
	public double getSmoothedDerivative() {
		return (current_ - prev1_) / timeStep_;
	}

	@Override
	public double getSmoothedSecondDerivative() {
		double curDeriv = (current_ - prev1_) / timeStep_;
		double prevDeriv = (prev1_ - prev2_) / timeStep_;
		double secondDeriv = (curDeriv - prevDeriv) / timeStep_;
		return secondDeriv;
	}

	//////////////////
	// member elements
	//////////////////
	
	private double				current_;
	private double				prev1_;
	private double				prev2_;
	private double				timeStep_;
}
