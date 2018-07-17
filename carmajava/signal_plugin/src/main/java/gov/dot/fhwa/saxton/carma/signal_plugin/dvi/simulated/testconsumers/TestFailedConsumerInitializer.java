package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;

/**
 * An initializer that returns false
 */
public class TestFailedConsumerInitializer  implements IConsumerInitializer {
    @Override
    public Boolean call() throws Exception {
        return new Boolean(false);
    }
}
