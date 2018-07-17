package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;

public interface ISpatMessage extends IAsdMessage {

	DataElementHolder getSpatForLane(int lane);
}
