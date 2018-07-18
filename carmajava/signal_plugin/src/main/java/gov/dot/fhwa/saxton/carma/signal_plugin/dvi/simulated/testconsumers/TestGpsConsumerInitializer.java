package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Skeleton GpsConsumerInitializer
 */
public class TestGpsConsumerInitializer  implements IConsumerInitializer {

    ILogger logger = LoggerManager.getLogger(TestGpsConsumerInitializer.class);

    @Override
    public Boolean call() throws Exception {
        logger.debug("CON", "Executing TestGpsConsumerInitializer call()");
        return new Boolean(true);
    }
}
