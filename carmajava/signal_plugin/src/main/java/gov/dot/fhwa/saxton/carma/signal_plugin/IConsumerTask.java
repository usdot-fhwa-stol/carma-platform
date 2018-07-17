package gov.dot.fhwa.saxton.glidepath;


import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;

import java.util.concurrent.Callable;

/**
 * An IConsumerTask is a consumer for one of the devices
 *
 * The executor holds the consumers in a list
 * It will acquire each consumers initializer
 *
 * The sequence the Executor follows is:
 *
 *      bResult = consumer.initialize()
 *      if (false)
 *          consumer.terminate();
 *
 *      IConsumerInitializer initializer = consumer.getInitializer();
 *
 *      Future<Boolean> future = executor.submit(initializer);
 *
 *      The executor will treat false or exception the same and just call consumer.terminate();
 *
 *
 */
public interface IConsumerTask extends Callable<DataElementHolder>  {
    public boolean initialize();
    public void terminate();
    public IConsumerInitializer getInitializer();
}
