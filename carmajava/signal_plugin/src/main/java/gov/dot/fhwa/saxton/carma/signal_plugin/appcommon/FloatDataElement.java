package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;


import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElement;

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
