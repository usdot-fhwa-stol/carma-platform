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

package gov.dot.fhwa.saxton.carma.signal_plugin.asd;

import java.util.List;

/**
 * Collection of all known intersections at the current time.
 */
public class IntersectionCollection {
	
    private List<IntersectionData> intersections; //ordered from nearest (element 0) to farthest from vehicle

	public List<IntersectionData> getIntersections() {
		return intersections;
	}

	public void setIntersections(List<IntersectionData> intersections) {
		this.intersections = intersections;
	}
    
}
