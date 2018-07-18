package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

public abstract class DataElement {
	
	public					DataElement(){
		//records the time the element is created
		
		timeStamp_ = System.currentTimeMillis();
	}

	public long				timeStamp(){ 
		//time the data was created in ms
		return timeStamp_;
	}
	
	////////////////////////////////////////////////////
	
	protected long			timeStamp_;	//time the data element was created in ms from 1/1/1970
}
