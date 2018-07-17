package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;

/**
 * An initializer that throws an exception
 */
public class TestExceptionConsumerInitializer  implements IConsumerInitializer {
    @Override
    public Boolean call() throws Exception {
        throw new Exception("Iniitializer throwing Exception.");
    }
}
