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

/**
 * Interface to a data filtering class that is intended to be used on sequentially generated data points
 * as they are generated. The implementing algorithm will take in one data point at a time (the newest
 * one in an ongoing sequence), and combine it with data points provided in previous calls to smooth out
 * the noise in the signal.
 * 
 * ASSUMES that data points are evenly spaced at a uniform time step separation
 * 
 * @author starkj
 *
 */
public interface IDataFilter {

	/**
	 * stores the time step size, in seconds, and initializes the model (this could be done in a constructor,
	 * but having this method will force every implementing class to recognize the need to do it).
	 */
	public void initialize(double timeStep);
	
	/**
	 * takes in the most recent raw data point, combines it with historical points (from previous calls)
	 */
	public void addRawDataPoint(double rawValue);
	
	/**
	 * returns a revised value for the current time step.  The revised value is intended to be used
	 * by the caller in place of the provided raw data. 
	 */
	public double getSmoothedValue();
	
	/**
	 * returns a smoothed derivative of the raw data stream for the current time step.
	 */
	public double getSmoothedDerivative();
	
	/**
	 * returns a smoothed second derivative of the raw data stream for the current time step.
	 * @return
	 */
	public double getSmoothedSecondDerivative();
}
