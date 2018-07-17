package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;

import java.util.List;

public class IAsdListDataElement extends DataElement {

    public IAsdListDataElement(List<IAsdMessage> val) {
        super();
        value_ = val;
    }

    public List<IAsdMessage> value() {
        return value_;
    }

    ////////////////////////////

    protected List<IAsdMessage> value_;
}
