package gov.dot.fhwa.saxton.carma.signal_plugin;


import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;

public interface IProducerTask {
    public DataElementHolder produce(DataElementHolder holder);
}
