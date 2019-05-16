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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.ISpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IntersectionGeometry;

/**
 * Collector for all data relative to a single intersections that needs to be passed through the system.
 *
 * Note that objects of this class will be constructed piecemeal, so at any given time some of these attributes
 * will be undefined. In particular, an IntersectionData will typically be created to hold only a map or a spat
 * attribute, and the other elements will (or may) be added later in the timestep or even in a future timestep.
 */
public class IntersectionData {
    //these are inputs to the Trajectory class
    private MapMessage              map = null;
    private ISpatMessage            spat = null;
    //these are for internal use by the Trajectory class
    private int                     roughDist = Integer.MAX_VALUE; //straight-line dist from vehicle to ref point, cm
    private int                     missingTimesteps = 0; //num consecutive time steps since last MAP/SPAT data received
    //these are outputs from the Trajectory class
    private int                     intersectionId = -1; //can be gotten from map above
    private int                     laneId = -1;
    private double                  dtsb; //distance to stop bar along the approach lane, in meters
    private SignalPhase             currentPhase = SignalPhase.NONE;
    private double                  timeToNextPhase = -1.0;
    private IntersectionGeometry    geometry = null;
    private double                  stopBoxWidth = -1.0; // This will only be set if geometry is not null

    // TODO comment
    public double bestDTSB() {
        if (geometry == null) { // If there is no geometry than dtsb will not be set
            return (0.01 * (double)roughDist); // Use rough dist if geometry is not available
        } else {
            return dtsb; // Use dtsb if available
        }
    }
    
    @Override
    public String toString() {
        return "IntersectionData [roughDist=" + roughDist + ", missingTimesteps="
                + missingTimesteps + ", intersectionId=" + intersectionId + ", laneId=" + laneId + ", dtsb=" + dtsb
                + ", currentPhase=" + currentPhase + ", timeToNextPhase=" + timeToNextPhase + "]";
    }

	public MapMessage getMap() {
		return map;
	}

	public void setMap(MapMessage map) {
		this.map = map;
	}

	public ISpatMessage getSpat() {
		return spat;
	}

	public void setSpat(ISpatMessage spat) {
		this.spat = spat;
	}

	public int getIntersectionId() {
		return intersectionId;
	}

	public void setIntersectionId(int intersectionId) {
		this.intersectionId = intersectionId;
	}

	public int getLaneId() {
		return laneId;
	}

	public void setLaneId(int laneId) {
		this.laneId = laneId;
	}

	public double getDtsb() {
		return dtsb;
	}

	public void setDtsb(double dtsb) {
		this.dtsb = dtsb;
	}

	public SignalPhase getCurrentPhase() {
		return currentPhase;
	}

	public void setCurrentPhase(SignalPhase currentPhase) {
		this.currentPhase = currentPhase;
	}

	public double getTimeToNextPhase() {
		return timeToNextPhase;
	}

	public void setTimeToNextPhase(double timeToNextPhase) {
		this.timeToNextPhase = timeToNextPhase;
	}

	public int getRoughDist() {
		return roughDist;
	}

	public void setRoughDist(int roughDist) {
		this.roughDist = roughDist;
	}

	public IntersectionGeometry getGeometry() {
		return geometry;
	}

	public void setGeometry(IntersectionGeometry geometry) {
		this.geometry = geometry;
	}

	public double getStopBoxWidth() {
		return stopBoxWidth;
	}

	public void setStopBoxWidth(double stopBoxWidth) {
		this.stopBoxWidth = stopBoxWidth;
	}
	
}
