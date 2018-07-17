package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;

import java.io.IOException;

/**
 * Skeleton CanConsumer
 */
public class TestCanConsumer implements IConsumerTask {

    private IConsumerInitializer initializer = new TestCanConsumerInitializer();

    public TestCanConsumer()   {
        // left blank
    }

    // provide flexibility to init with a failing initializer
    public TestCanConsumer(IConsumerInitializer initializer)   {
        this.initializer = initializer;
    }


    @Override
    public boolean initialize() {
        return true;
    }

    @Override
    public void terminate() {
        //To change body of implemented methods use File | Settings | File Templates.
    }

    public IConsumerInitializer getInitializer()   {
        return initializer;
    }

    public DataElementHolder call() throws IOException {
        DataElementHolder holder = new DataElementHolder();

        return holder;
    }

}