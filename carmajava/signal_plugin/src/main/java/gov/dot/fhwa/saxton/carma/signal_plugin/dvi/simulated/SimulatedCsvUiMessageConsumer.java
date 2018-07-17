package gov.dot.fhwa.saxton.glidepath.dvi.simulated;

import gov.dot.fhwa.saxton.glidepath.dvi.SpeedControl;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.DviUIMessage;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathStateModel;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.io.IOException;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * SimulatedCsvUiMessageConsumer
 *
 * Simulates our Consumer model for consuming/reading info from a device
 *
 * This simplified class only has the periodic task defined as a Callable.  For device consuming,
 * both the initialization task and terminate task should be implement callable.
 *
 * TODO: Define our domain objects (wrap with DataElement, etc.) and callable returns.
 *
 */
public class SimulatedCsvUiMessageConsumer implements Callable {

    ILogger logger = LoggerManager.getLogger(SpeedControl.class);

    private static final String uiMessageFile = "testdata/uiMessages.csv";

    // this executor is just used to submit initialize and shutdown
    private ExecutorService executor = Executors.newFixedThreadPool(2);

    // NOTE: These aren't needed in the simulated environment.
    // Device environment these tasked should implement Callable
    private Callable initializer;
    private Callable terminator;


    CsvSimulatedUiMessageProducer producer = new CsvSimulatedUiMessageProducer(uiMessageFile);

    public SimulatedCsvUiMessageConsumer()   {
        // intentionally left blank
    }

    @Override
    public DviUIMessage call() {
        DviUIMessage uiMessage = producer.getUiMessage();
        uiMessage.setGlidepathState(GlidepathStateModel.getInstance().getState());
        return uiMessage;
    }

    public boolean initialize()   {
        producer.load();
        logger.infof("TAG", "Consumer initializing...");
        try   {
            LoggerManager.writeToDisk();
        }
        catch(IOException ioe)   {
            System.err.println("Error writing log to disk: " + ioe.getMessage());
        }
        return true;
    }


    public boolean terminate()   {
        logger.infof("TAG", "Consumer terminating...");
        try   {
            LoggerManager.writeToDisk();
        }
        catch(IOException ioe)   {
            System.err.println("Error writing log to disk: " + ioe.getMessage());
        }
        return true;
    }

}
