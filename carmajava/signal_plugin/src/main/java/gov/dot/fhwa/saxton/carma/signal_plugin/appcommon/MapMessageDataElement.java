package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;

public class MapMessageDataElement extends DataElement {

	public MapMessageDataElement(MapMessage val) {
		super();
		value_ = val;
	}
	
	public MapMessage value() {
		return value_;
	}
	
	////////////////////////////
	
	protected MapMessage value_;
}
