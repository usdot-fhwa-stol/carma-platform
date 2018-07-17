package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.MotionStatus;

public class MotionStatusDataElement extends DataElement {

    public MotionStatusDataElement(MotionStatus val){
        super();
        value_ = val;
    }

    public MotionStatus				value(){
        //returns the value of the data element

        return value_;
    }

    ////////////////////////////////////////////////

    protected MotionStatus value_; //element value
}