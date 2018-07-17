package gov.dot.fhwa.saxton.glidepath.asd;

import gov.dot.fhwa.saxton.glidepath.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.glidepath.asd.map.MapMessage;
import gov.dot.fhwa.saxton.glidepath.asd.spat.ISpatMessage;

/**
 * Collector for all data relative to a single intersections that needs to be passed through the system.
 *
 * Note that objects of this class will be constructed piecemeal, so at any given time some of these attributes
 * will be undefined. In particular, an IntersectionData will typically be created to hold only a map or a spat
 * attribute, and the other elements will (or may) be added later in the timestep or even in a future timestep.
 */
public class IntersectionData {
    //these are inputs to the Trajectory class
    public MapMessage              map = null;
    public ISpatMessage            spat = null;
    //these are for internal use by the Trajectory class
    public int                     roughDist = Integer.MAX_VALUE; //straight-line dist from vehicle to ref point, cm
    public int                     missingTimesteps = 0; //num consecutive time steps since last MAP/SPAT data received
    //these are outputs from the Trajectory class
    public int                     intersectionId = -1; //can be gotten from map above
    public int                     laneId = -1;
    public double                  dtsb; //distance to stop bar along the approach lane, in meters
    public SignalPhase             currentPhase = SignalPhase.NONE;
    public double                  timeToNextPhase = -1.0;
    public double                  timeToThirdPhase = -1.0;
}
