package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;

/**
 * Skeleton CanConsumerInitializer
 */
public class TestCanConsumerInitializer  implements IConsumerInitializer {
    @Override
    public Boolean call() throws Exception {
        return new Boolean(true);
    }
}
