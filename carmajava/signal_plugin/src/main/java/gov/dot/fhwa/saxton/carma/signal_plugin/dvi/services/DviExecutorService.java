package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.services;

import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.appcommon.*;
import gov.dot.fhwa.saxton.glidepath.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.glidepath.dvi.GlidepathAppConfig;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.DviParameters;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.DviUIMessage;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathState;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathStateModel;
import gov.dot.fhwa.saxton.glidepath.ead.ITrajectory;
import gov.dot.fhwa.saxton.glidepath.ead.TrajectoryFactory;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LogEntry;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvStatus;
import org.joda.time.DateTime;
import org.joda.time.Duration;
import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import java.io.File;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The executor service that manages the device read cycles
 *
 */
@Service
public class DviExecutorService implements Callable<Boolean>, DisposableBean {

    private static ILogger logger = LoggerManager.getLogger(DviExecutorService.class);

    private AtomicBoolean bShutdown = new AtomicBoolean(true);
    private static int refreshCounter = 0;
    private int uiRefresh = 10;

    private AtomicBoolean bRecordData = new AtomicBoolean(true);

    @Autowired
    private GlidepathAppConfig appConfig;

    @Autowired
    private SimpMessagingTemplate template;

    private List<IConsumerTask> consumers;

    private ExecutorService executor;

    private PeriodicExecutor periodicExecutor;

    // Used to stop app after rolling logs
    private final ScheduledExecutorService scheduledExecutor = Executors.newScheduledThreadPool(1);

    private ITrajectory trajectory;

    private int readCycle;

    /**
     * Adjustment to tweak sleep time at end of cycle, defaults to 5ms, but uses sleep.adjustment if configured
     */
    private int sleepAdjustment = 0;

    /**
     * Starts the device consumer threads
     *
     * @return
     */
    public boolean start()   {
        setLogLevel();

        boolean result = false;

        bShutdown = new AtomicBoolean(false);
        LoggerManager.setRecordData(true);

        // set maximum speed from configuration
        DviParameters dviParameters = DviParameters.getInstance();
        dviParameters.setMaximumSpeed(appConfig.getMaximumSpeed());

        readCycle = appConfig.getPeriodicDelay();
        uiRefresh = appConfig.getUiRefresh();
        sleepAdjustment = appConfig.getDefaultIntValue("sleep.adjustment", sleepAdjustment);

        try   {
            String trajectoryClassName = appConfig.getProperty("ead.trajectoryclass");
            trajectoryClassName = (trajectoryClassName == null) ? "gov.dot.fhwa.saxton.glidepath.ead.SimulatedTrajectory" : trajectoryClassName;
            trajectory = TrajectoryFactory.newInstance(trajectoryClassName);
            if (trajectory == null) {
                throw new Exception("DviExecutor: unable to instantiate the trajectory object.");
            }
        }
        catch(Exception e)   {
            logger.error(ILogger.TAG_EXECUTOR, "Error loading trajectory: " + e.getMessage());
            return false;
        }

        executor = Executors.newFixedThreadPool(2);

        try   {
            // allow setting any consumers to better support testing
            if (consumers == null || consumers.isEmpty())   {
                consumers = ConsumerFactory.getConsumers();
            }

            boolean bStateChange = GlidepathStateModel.getInstance().changeState(GlidepathState.SETUP, null);

            DeviceInitializer deviceInitializer = new DeviceInitializer(consumers);
            result = deviceInitializer.start();
            if (result)   {
                logger.output(ILogger.TAG_EXECUTOR, DataElementHolder.getLogHeader());

                bStateChange = GlidepathStateModel.getInstance().changeState(GlidepathState.STANDBY, null);

                periodicExecutor = new PeriodicExecutor(trajectory);
                periodicExecutor.addConsumers(consumers);
                executor.submit(this);
            }
            else   {
                // a consumer failed to initialize, stop the app
                try   {
                    LoggerManager.writeToDisk();
                }
                catch(Exception e)   {
                    System.err.println("Error writing to log file:" + e.getMessage());
                }

                autoRollLogs();
            }
        }
        catch(Exception e)   {
            result = false;
            logger.error(ILogger.TAG_EXECUTOR, "Error starting DeviceInitializer: " + e.getMessage());
        }

        return result;
    }

    /**
     * The read cycle loop
     *
     * Uses the periodicExecutor to submit each device read.  The response from the periodic executor
     * is what needs to be provided to the DVI clients
     *
     * @return
     */
    public Boolean call()   {
        DataElementHolder lastHolder = null;
        int lastIntersectionId = 0;
        boolean debugMode = GlidepathApplicationContext.getInstance().getAppConfig().getBooleanValue("debug");

        while (!bShutdown.get())   {
            logger.debug(ILogger.TAG_EXECUTOR, "");
            logger.debug("CYCLE", "****#### Start of Consumer Cycle ####****");

            DateTime startTime = new DateTime();

            DataElementHolder holder = new DataElementHolder();

            //invoke the periodic executor for a single time step
            Future<DataElementHolder> future = executor.submit(periodicExecutor);
            if (future == null) {
                logger.error(ILogger.TAG_EXECUTOR, "Unable to retrieve a future from periodic executor.");
                return false;
            }
            logger.debug(ILogger.TAG_EXECUTOR, "future object has been defined.");

            try   {
                holder = future.get();
                logger.debug(ILogger.TAG_EXECUTOR, "future object has returned a new holder.");
                if (holder == null) {
                    logger.error(ILogger.TAG_EXECUTOR, "Valid future unable to provide a holder object.");
                    return false;
                }

                //if no intersections is available then display a 0 for the ID
                IntDataElement intersectionIdElement = (IntDataElement) holder.get(DataElementKey.INTERSECTION_ID);
                if (intersectionIdElement == null)   {
                    logger.info(ILogger.TAG_EXECUTOR, "No intersection ID available in incoming holder. Assigning ID=0");
                    holder.put(DataElementKey.INTERSECTION_ID, new IntDataElement(0));
                }
                logger.debug(ILogger.TAG_EXECUTOR, "intersectionIdElement has been established.");

                // only record consumer data result if flag is set
                if (bRecordData.get())   {
                    logger.info("", "###### holder size: " + holder.size() + " ******");
                    logger.output(ILogger.TAG_EXECUTOR, holder.getLogString());
                    logger.debug(ILogger.TAG_EXECUTOR, "holder has been logged to string");
                    outputXgvStatus(holder);
                }
                logger.debug(ILogger.TAG_EXECUTOR, "Ready to prepare the UI message.");

                // provide the dvi ui message IF we have debug set OR we have a fully loaded holder
                if (debugMode || holder.validate() )   {

                    //debugging XGV indicator
                    XgvStatusDataElement xse = new XgvStatusDataElement(holder.getXgvStatus());
                    XgvStatus xs = xse.value();
                    //logger.debugf("EXEC", "Sending xgv status to DVI: key=%b, gear=%s, brake=%b, safestop=%b, override=%b",
                    //            xs.isOn(), xs.getGear().toString(), xs.isParkingBrakeSet(), xs.isSafeStopStopEngaged(),
                    //            xs.isManualOverrideEngaged());


                    logger.debug(ILogger.TAG_EXECUTOR, "Ready to set DviUiMessage.");
                    DviUIMessage dviUiMessage = new DviUIMessage(holder);
                    if (dviUiMessage == null) {
                        logger.warn(ILogger.TAG_EXECUTOR, "Failed to create UI message.");
                    }else {
                        setDviUiMessage(dviUiMessage);
                    }
                }
            }
            catch(Exception e)   {
                // error
                logger.error(ILogger.TAG_EXECUTOR, "Exception getting periodicExecutor future: " + e.getMessage());
                e.printStackTrace();
            }

            DateTime endTime = new DateTime();
            Duration dur = new Duration(startTime, endTime);
            long cycleLasted = dur.getMillis();

            if (bRecordData.get())   {
                logger.debug(ILogger.TAG_EXECUTOR, "Full consumer read cycle in Millis: " + cycleLasted);
            }

            // provide configuration capability to adjust sleep time to help obtain closer readCycles exactly equal to readCycle
            if (cycleLasted <= readCycle - sleepAdjustment)   {
                try   {
                    Thread.sleep(readCycle - cycleLasted - sleepAdjustment);
                }
                catch(InterruptedException ie)   {
                }
            }
            if (cycleLasted > readCycle  &&  bRecordData.get()){
                logger.warn(ILogger.TAG_EXECUTOR, "#### Read cycle exceeded periodicDelay ####: " + cycleLasted);
            }

            try   { LoggerManager.writeToDisk(); }
            catch(Exception e)   {}

        }


        for (IConsumerTask consumer : consumers)   {
            consumer.terminate();
        }

        // when we close the app, we are mostly likely last in ENGAGED state
        //   setting back to state1 STARTUP for Apache purposes
        GlidepathStateModel.getInstance().changeState(GlidepathState.STARTUP, null);

        periodicExecutor.stop();
        trajectory.close();
        executor.shutdown();

        try   {
            LoggerManager.writeToDisk();
        }
        catch(Exception e)   {
            System.err.println("Error writing to log file:" + e.getMessage());
        }

        return new Boolean(true);
    }

    @Override
    public void destroy()   {
        stop();

        try   {
            Thread.sleep(100);
        }
        catch(Exception e) {};
        logger.info(ILogger.TAG_EXECUTOR,  "Destroying bean DviExecutorService via lifecycle destroy().");
    }

    /**
     * Signal to exit the read cycle loop
     */
    public void stop()   {
        bShutdown.getAndSet(true);
    }

    protected void printConfig()   {
        logger.info("", appConfig.toString());
    }

    /**
     * Configuration to indicate whether we should start reading devices on startup
     *
     * @return boolean indicating whether to start threads on startup
     */
    public boolean getAutoStartConsumption()   {
        return appConfig.getAutoStartConsumption();
    }

    /**
     * Provide direct setting of consumer list
     *
     * @param consumers
     */
    public void setConsumers(List<IConsumerTask> consumers)   {
        this.consumers = consumers;
    }


    /**
     * Sends message to clients subscribing to our stomp/websockets endpoints
     *
     * @param uiMessage
     */
    public synchronized void setDviUiMessage(DviUIMessage uiMessage)   {
        logger.debug(ILogger.TAG_EXECUTOR,  "Setting DomainObject from consumer: " + uiMessage.toString());
        if (refreshCounter >= uiRefresh - 1)   {
            template.convertAndSend("/topic/dvitopic", uiMessage);
            refreshCounter = 0;
        }
        else   {
            refreshCounter += 1;
        }
    }

    /**
     * Retrieve the state of the record data flag
     * @return
     */
    public boolean getRecordData()   {
        return bRecordData.get();
    }

    /**
     * Set whether we want consumer data logging to begin or stop
     *
     * @param flag
     */
    public void setRecordData(boolean flag)   {
        bRecordData.getAndSet(flag);
        LoggerManager.setRecordData(flag);
    }

    /**
     * Log XGV status members as tab delimited to ease debugging
     *
     * @param holder
     */
    private void outputXgvStatus(DataElementHolder holder)   {

        if (holder != null)   {
            XgvStatus xgvStatus = holder.getXgvStatus();

            if (xgvStatus != null)   {
                String xgvMessage = xgvStatus.toString().replaceAll(",", "\t");
                logger.debug("XSF", xgvMessage);
            }
        }
    }



    /**
     * Rename log files to start new recording
     *
     * @return boolean
     */
    public boolean moveLogs()   {
        boolean result = false;

        try   {
            File logFile = new File(Constants.LOG_FILE);
            result = logFile.renameTo(new File(appConfig.getProperty("log.path") + "/" + DviParameters.getInstance().getLogFileName()));
        }
        catch(Exception e)   {
            logger.error(ILogger.TAG_EXECUTOR, "Error moving file: " + e.getMessage());
        }

        return result;
    }

    /**
     * Set min log level
     *
     * If not configured or configured incorrectly, uses DEBUG
     */
    public void setLogLevel()   {
        String logLevel = appConfig.getProperty("log.level");

        LogEntry.Level enumLevel = null;

        try   {
            enumLevel = LogEntry.Level.valueOf(logLevel.toUpperCase());
        }
        catch(Exception e)   {
            logger.warn("EXEC", "log.level value improperly configured: " + logLevel);
            enumLevel = LogEntry.Level.DEBUG;
        }

        LoggerManager.setMinOutputToWrite(enumLevel);
    }

    /**
     * Stop dvi services and roll the logs.  Additionally, schedule an app stop in two seconds.  This gives the client
     * a response so that they can navigate to a new page via javascript
     *
     * @return boolean
     */
    public boolean autoRollLogs()   {

        stop();
        // let things stop gracefully
        try   {
            Thread.sleep(1000);
        }
        catch(Exception e) {};
        setRecordData(false);

        boolean bResult = moveLogs();

        scheduledExecutor.schedule(new AppStop(), 3, TimeUnit.SECONDS);

        return bResult;
    }

}
