package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

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
