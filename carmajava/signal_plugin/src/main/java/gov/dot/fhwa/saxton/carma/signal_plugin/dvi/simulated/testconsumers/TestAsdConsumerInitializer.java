package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

/**
 * Skeleton CanConsumerInitializer
 */
public class TestAsdConsumerInitializer  implements IConsumerInitializer {

    private ILogger logger = LoggerManager.getLogger(TestAsdConsumerInitializer.class);

    @Override
    public Boolean call() throws Exception {
        logger.debug("CON", "Executing TestAsdConsumerInitializer call()");
        return new Boolean(true);
    }
}
