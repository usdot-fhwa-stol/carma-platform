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

package gov.dot.fhwa.saxton.carma.signal_plugin.filter;


import gov.dot.fhwa.saxton.carma.signal_plugin.filter.IDataFilter;

/**
 * A weighted moving average filter using 6 historical points.  Weights are linear starting with
 * 6 on the most recent point to 1 on the oldest point.
 * 
 * @author starkj
 *
 */
public class Wma6Filter implements IDataFilter {
	
	@Override
	public void initialize(double timeStep) {
		timeStep_ = timeStep;
		numPoints_ = 0;
		totalWeight_ = 0.0;
		next_ = SIZE - 1;		//start filling the buffer on the right end
		raw_ = new double[SIZE];
	}
	
	@Override
	public void addRawDataPoint(double rawValue) {
		addToHistory(rawValue);
	}

	@Override
	public double getSmoothedValue() {
		double smoothed = smooth();
		return smoothed;
	}
	
	@Override
	public double getSmoothedDerivative() {
		return 0.0; //TODO: bogus
	}
	
	@Override
	public double getSmoothedSecondDerivative() {
		return 0.0; //TODO: bogus
	}

	//////////////////
	// member elements
	//////////////////
	
	/**
	 * always : adds raw to the historical record of raw data points
	 * 
	 * Note: we will fill the buffer from right to left to avoid having to put an if test inside the smooth() loop.
	 * When the loop counter is counting up we can use modulo instead.
	 */
	private void addToHistory(double raw) {
		raw_[next_--] = raw;
		if (next_ < 0) {
			next_ = SIZE - 1;
		}
		if (numPoints_ < SIZE) {
			totalWeight_ += (double)(SIZE - numPoints_);
			++numPoints_;
		}
	}

	/**
	 * always : computes the smoothed value to replace the latest raw point
	 * 
	 * Note: guaranteed by caller to always be called after addToHistory()
	 */
	private double smooth() {
		double sum = 0.0;
		double weight;
		
		for (int i = SIZE - numPoints_;  i < SIZE;  ++i) { 
			weight = (double)((next_ - i + SIZE) % SIZE + 1); //next_ represents the index of the oldest data point
			sum += weight * raw_[i];
		}
		
		return sum / totalWeight_;
	}
	
	private int					numPoints_;
	private double				totalWeight_;
	private int					next_;
	private double[]			raw_;
	private double				timeStep_;
	
	private static final int	SIZE = 6;
}
