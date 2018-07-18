package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Skeleton XgvConsumerInitializer
 */
public class TestXgvConsumerInitializer  implements IConsumerInitializer {

    ILogger logger = LoggerManager.getLogger(TestXgvConsumerInitializer.class);

    @Override
    public Boolean call() throws Exception {
        logger.debug("CON", "Executing TestXvgConsumerInitializer call()");
        return new Boolean(true);
    }
}
