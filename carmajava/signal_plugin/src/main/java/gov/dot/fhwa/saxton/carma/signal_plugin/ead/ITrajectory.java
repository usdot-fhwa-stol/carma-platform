package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;

/**
 * Interface for an EAD wrapper
 * User: ferenced
 * Date: 1/14/15
 * Time: 1:20 PM
 */
public interface ITrajectory {
	public void engage();
    public boolean isStopConfirmed();
    public DataElementHolder getSpeedCommand(DataElementHolder state) throws Exception;
    public void close();
}
