package gov.dot.fhwa.saxton.glidepath.appcommon;

public class PhaseDataElement extends DataElement {

	public PhaseDataElement(SignalPhase val){
		super();
		value_ = val;
	}
	
	public SignalPhase				value(){
		//returns the value of the data element
		
		return value_;
	}
	
	////////////////////////////////////////////////
	
	protected SignalPhase value_; //element value
}
