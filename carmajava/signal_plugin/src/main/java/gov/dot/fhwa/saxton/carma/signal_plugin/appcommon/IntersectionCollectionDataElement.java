package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.glidepath.asd.IntersectionCollection;

public class IntersectionCollectionDataElement extends DataElement {

    public IntersectionCollectionDataElement(IntersectionCollection val) {
        super();
        value_ = val;
    }

    public IntersectionCollection value() {
        return value_;
    }

    ////////////////////////////

    protected IntersectionCollection value_;
}
