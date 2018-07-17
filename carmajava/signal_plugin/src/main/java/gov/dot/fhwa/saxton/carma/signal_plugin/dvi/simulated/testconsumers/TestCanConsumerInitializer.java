package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;

/**
 * Skeleton CanConsumerInitializer
 */
public class TestCanConsumerInitializer  implements IConsumerInitializer {
    @Override
    public Boolean call() throws Exception {
        return new Boolean(true);
    }
}
