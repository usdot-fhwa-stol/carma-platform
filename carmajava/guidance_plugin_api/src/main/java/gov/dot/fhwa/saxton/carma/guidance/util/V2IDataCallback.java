package gov.dot.fhwa.saxton.carma.guidance.util;

import java.util.List;

/**
 * Functional interface for callbacks to receive new V2I data from DSRC
 */
public interface V2IDataCallback {
    /**
     * Callback to be invoked when Guidance's V2I dataset is changed
     * 
     * @param data The new V2I data that has been updated
     */
    void onV2IDataChanged(List<IntersectionData> data);
}