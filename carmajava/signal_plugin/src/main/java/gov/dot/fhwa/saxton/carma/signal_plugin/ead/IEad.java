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

	// TODO comment
	List<Node> plan(double speed, double operSpeed, double accel,
                                 List<IntersectionData> intersections) throws Exception;


	// /**
	//  * Notifies the EAD object that the list of intersections has changed in some significant way
	//  * since the previous time step.
	//  */
	// void intersectionListHasChanged();
	
}
