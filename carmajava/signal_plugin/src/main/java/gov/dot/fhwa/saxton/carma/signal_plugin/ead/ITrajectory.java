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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import java.util.List;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * Interface for an EAD wrapper
 * User: ferenced
 * Date: 1/14/15
 * Time: 1:20 PM
 */
public interface ITrajectory {
	public void engage();
    public boolean isStopConfirmed();
    
    /**
     * Generates a plan based on the input data
     * 
     * @param stateData The current vehicle state data
		 * The following elements must be present in the vehicle state data
		 * - SMOOTHED_SPEED
		 * - ACCELERATION
		 * - OPERATING_SPEED
		 * - LATITUDE
		 * - LONGITUDE
		 * - INTERSECTION_COLLECTION
		 * 
     * @return A list of node defining the planned vehicle trajectory
     */
    public List<Node> plan(DataElementHolder stateData) throws Exception;
    public void close();
}
