package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

import gov.dot.fhwa.saxton.glidepath.asd.spat.ISpatMessage;

/**
 * Date Element containing a SPAT message
 *
 * User: ferenced
 * Date: 1/19/15
 * Time: 11:18 AM
 *
 */
public class SpatMessageDataElement extends DataElement {

    public SpatMessageDataElement(ISpatMessage val) {
        super();
        value_ = val;
    }

    public ISpatMessage value() {
        return value_;
    }

    ////////////////////////////

    protected ISpatMessage value_;
}
