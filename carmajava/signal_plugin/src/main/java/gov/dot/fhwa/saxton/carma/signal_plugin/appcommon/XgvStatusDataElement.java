package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.glidepath.xgv.XgvStatus;

/**
 * Surrounds an XgvStatus object in a DataElement container
 */
public class XgvStatusDataElement extends DataElement {
    XgvStatus data;

    public XgvStatusDataElement(XgvStatus status) {
        super();
        this.data = status;
    }

    public XgvStatus value() {
        return this.data;
    }

}
