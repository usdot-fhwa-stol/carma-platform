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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

// Interface to any of the possible EAD algorithm classes that we may choose to use.

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ITreeSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

import java.util.List;

public interface IEad {

	/**
	 * Provides one-time initialization of the EAD algorithm object.
	 * @param timestep - duration of system time step, ms
	 * @param solver - the tree solver to use
	 */
	void initialize(long timestep, ITreeSolver solver);

	/**
	 * Allows specification of the longitudinal width of the intersections's stop box.
	 * @param width - width of the box, m
	 */
	void setStopBoxWidth(double width);

	/**
	 * Generates a plan based on the input data
	 * 
	 * @param speed The current speed of the vehicle
	 * @param operSpeed The target operating speed of the vehicle
	 * @param intersections A sorted list of intersection data with the nearest intersections appearing earlier in the list
	 * @param startTime The time which planning is considered to have begun at. This is used for converting nodes to route locations
	 * @param startDowntrack The downtrack distance where planning is considered to have begun at. This is used for converting nodes to route locations
	 * 
	 * @return A list of node defining the planned vehicle trajectory
	 */
	List<Node> plan(double speed, double operSpeed, 
		List<IntersectionData> intersections, double startTime, double startDowntrack) throws Exception;
	
}
