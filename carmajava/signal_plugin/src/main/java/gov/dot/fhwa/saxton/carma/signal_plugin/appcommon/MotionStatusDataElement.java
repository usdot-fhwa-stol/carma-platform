package gov.dot.fhwa.saxton.glidepath.appcommon;

import gov.dot.fhwa.saxton.glidepath.dvi.domain.MotionStatus;

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