package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated;


//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;
//
//import java.util.concurrent.Callable;
//import java.util.concurrent.ExecutorService;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathState;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathStateModel;
import gov.dot.fhwa.saxton.glidepath.dvi.simulated.SimulatedCsvUiMessageConsumer;
import gov.dot.fhwa.saxton.glidepath.dvi.simulated.SimulatedDviExecutorService;

import java.util.concurrent.Executors;
//import java.util.concurrent.Future;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * SimulatedUiMessagePeriodicExecutor
 *
 * Initializes DviUIMessage Consumer and periodically executes consumption of one record
 */
public class SimulatedUiMessagePeriodicExecutor  {

    private boolean bShutdown = false;
    SimulatedDviExecutorService dviExecutor;
    SimulatedCsvUiMessageConsumer consumer;     // Consumer of DviUIMessages
    int delay;                                  // periodic delay in ms

    ScheduledThreadPoolExecutor scheduledExecutor;

    public SimulatedUiMessagePeriodicExecutor(SimulatedDviExecutorService dviExecutor, int delay)   {
        this.dviExecutor = dviExecutor;
        this.consumer = new SimulatedCsvUiMessageConsumer();
        this.delay = delay;
    }

    /**
     * initialize
     *
     * Initialize the consumer and schedule the periodic task
     *
     * @return
     */
    public boolean initialize()   {
        GlidepathStateModel.getInstance().changeState(GlidepathState.SETUP, null);
        boolean bResult = consumer.initialize();
        if (bResult)   {
            GlidepathStateModel.getInstance().changeState(GlidepathState.STANDBY, null);
            scheduledExecutor = (ScheduledThreadPoolExecutor) Executors.newScheduledThreadPool(5);
            scheduledExecutor.scheduleWithFixedDelay(new Runnable() {
                public void run() {
                    dviExecutor.setDviUiMessage(consumer.call());
                } },
                    1, delay, TimeUnit.MILLISECONDS);
        }
        return bResult;
    }

    /**
     * stop
     *
     * Terminate the consumer and shutdown the scheduled executor
     *
     * @return
     */
    public boolean stop()   {
        boolean bResult = consumer.terminate();
        scheduledExecutor.shutdown();
        bShutdown = true;
        return bResult;
    }


}
