package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerInitializer;
import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerTask;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IntDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.gps.simulated.SimulatedGpsProducer;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.io.IOException;

/**
 * Skeleton GpsConsumer
 */
public class TestGpsConsumer implements IConsumerTask {

    ILogger logger = LoggerManager.getLogger(TestGpsConsumer.class);

    private IConsumerInitializer initializer = new TestGpsConsumerInitializer();
    private SimulatedGpsProducer producer = new SimulatedGpsProducer("testdata/gpsGlobalPositions.csv");

    private static double latitude = 1.0;

    public TestGpsConsumer()   {
        // left blank
    }

    // provide flexibility to init with a failing initializer
    public TestGpsConsumer(IConsumerInitializer initializer)   {
        this.initializer = initializer;
    }

    @Override
    public boolean initialize() {
        producer.load();
        logger.debug("CON", "Initializing TestGpsConsumer");
        return true;
    }

    @Override
    public void terminate() {
        logger.debug("CON", "Terminating TestGpsConsumer");
    }

    public IConsumerInitializer getInitializer()   {
        return initializer;
    }

    public DataElementHolder call() throws IOException {
        DateTime startTime = new DateTime();

        DataElementHolder holder = producer.getGpsData();

        // the producer reads in the csv file and creates a holder when loaded which results in stale data
        // we will simply extract the data and create a new holder so that the data timestamp is no longer stale
        double latitude = holder.getLatitude();
        double longitude = holder.getLongitude();

        DataElementHolder newHolder = new DataElementHolder();
        newHolder.put(DataElementKey.LATITUDE, new DoubleDataElement(latitude));
        newHolder.put(DataElementKey.LONGITUDE, new DoubleDataElement(longitude));

        Duration duration = new Duration(startTime, new DateTime());
        newHolder.put(DataElementKey.CYCLE_GPS, new IntDataElement((int) duration.getMillis()));

        return newHolder;
    }

}