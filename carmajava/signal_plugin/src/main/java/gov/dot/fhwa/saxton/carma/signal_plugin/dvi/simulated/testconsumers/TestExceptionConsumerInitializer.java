package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;

/**
 * An initializer that throws an exception
 */
public class TestExceptionConsumerInitializer  implements IConsumerInitializer {
    @Override
    public Boolean call() throws Exception {
        throw new Exception("Iniitializer throwing Exception.");
    }
}
