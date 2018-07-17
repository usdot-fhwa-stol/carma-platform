package gov.dot.fhwa.saxton.glidepath.appcommon;


import gov.dot.fhwa.saxton.glidepath.appcommon.DataElement;

public class FloatDataElement extends DataElement {

    protected float value_;

    public FloatDataElement(float val)   {
        super();
        this.value_ = val;
    }

    public float value()   {
        return value_;
    }
}
