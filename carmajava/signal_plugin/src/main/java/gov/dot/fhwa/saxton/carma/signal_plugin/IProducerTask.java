package gov.dot.fhwa.saxton.glidepath;


import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;

public interface IProducerTask {
    public DataElementHolder produce(DataElementHolder holder);
}
