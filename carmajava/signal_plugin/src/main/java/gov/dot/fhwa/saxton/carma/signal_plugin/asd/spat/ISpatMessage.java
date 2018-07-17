package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;

public interface ISpatMessage extends IAsdMessage {

	DataElementHolder getSpatForLane(int lane);
}
