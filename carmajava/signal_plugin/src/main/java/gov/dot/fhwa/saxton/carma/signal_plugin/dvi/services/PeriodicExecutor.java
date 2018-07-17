package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.services;


import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.appcommon.*;
import gov.dot.fhwa.saxton.glidepath.appcommon.utils.ConversionUtils;
import gov.dot.fhwa.saxton.glidepath.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.IntersectionCollection;
import gov.dot.fhwa.saxton.glidepath.asd.IntersectionData;
import gov.dot.fhwa.saxton.glidepath.asd.map.MapMessage;
import gov.dot.fhwa.saxton.glidepath.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.glidepath.dvi.GlidepathAppConfig;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.DviParameters;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathState;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathStateModel;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.MotionStatus;
import gov.dot.fhwa.saxton.glidepath.ead.ITrajectory;
import gov.dot.fhwa.saxton.glidepath.filter.DataFilterFactory;
import gov.dot.fhwa.saxton.glidepath.filter.IDataFilter;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvConnection;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvConsumer;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvSpeedController;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvStatus;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * The PeriodicExecutor
 */
public class PeriodicExecutor implements Callable<DataElementHolder> {

    private static int loop = 0;

    private static ILogger logger = LoggerManager.getLogger(PeriodicExecutor.class);

    private List<IConsumerTask> consumers = new ArrayList<IConsumerTask>();

    private ExecutorService executor = Executors.newFixedThreadPool(5);

    private List<Future<DataElementHolder>> futures = new ArrayList<Future<DataElementHolder>>();

    private ITrajectory trajectory;

    private XgvSpeedController xgvSpeedController;

    // if not configured, use 5 cycles
    private int missingDataThreshold = 5;

    // flag used to indicate we have are engaged and have called ead.engaged()
    private boolean bEngaged = false;

    private double lastSpeed = 0.0;

    /**
     * This variable is used to ensure we maintain the information that XgvStatus.manualOverrideEngaged was
     * triggered once we start ecoDrive.
     *
     * As we are not providing every time step info to the UI, we need to know that manualOverrideEngaged was set to
     * true at any point once we are ENGAGED.  This way, the client is assured of getting this information even
     * when it occurs in a time step UIMessage not delivered to the client
     */
    private boolean bOverrideEngaged = false;

    private IDataFilter filter;

    /**
     * Maintains a queue of status messages that may result in a warning message to the DVI
     * The statusQueue is the compilation of status messages for the last cycleMax cycles
     * If we reach our threshold of cycleThreshold (3 warnings/etc. within 20 cycles), we will
     * use the previousStatusQueue to display those messages for the next cycleMax cycles
     *
     * If we reach cycleMax without accumulating cycleThreshold statusMessages, we use an empty
     * previousStatusQueue to no longer display status messages on the DVI
     *
     * Any Consumer can provide a status message to the DVI by creating a StringBufferDataElement using
     * DataElementKey.STATUS_MESSAGE
     *
     *  StringBuffer sb = new StringBuffer();
     *  sb.append("There was some error i wanna report to exec/DVI")
     *  holder.put(DataElementKey.STATUS_MESSAGE, new StringBufferDataElement(sb));
     *
     *  Multiple messages can be appended to the element either appending to the StringBuffer prior to creating the
     *  element or using the holder.appendStatusMessage if the element has already been created.
     *
     * Any STATUS_MESSAGE element provided by a Consumer is removed from that consumers holder and appended to the
     * Executors STATUS_MESSAGE element.
     */
    private Queue<String> statusQueue = new LinkedList<String>();
    private Queue<String> previousStatusQueue = new LinkedList<String>();
    private int cycle = 0;

    /**
     * Default values overridden if they appear in the dvi.properties file
     * The default configuration indicates if we have 3 statusMessages within 20 cycles, we will display the
     * messages in the DVI for the next 20 cycles
     */
    private int cycleThreshold = 3;
    private int cycleMax = 20;

    /**
     * Constructor with provided trajectory
     *
     * @param trajectory
     */
    public PeriodicExecutor(ITrajectory trajectory)   {
        this.trajectory = trajectory;
        String strThreshold = (GlidepathApplicationContext.getInstance().getAppConfig()).getProperty("missingDataThreshold");
        if (strThreshold != null)    {
            int nThreshold = Integer.parseInt(strThreshold);
            missingDataThreshold = ( nThreshold == 0 ) ? missingDataThreshold : nThreshold;
        }

        GlidepathAppConfig appConfig = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig());

        this.cycleMax = appConfig.getDefaultIntValue("cycleMax", this.cycleMax);
        this.cycleThreshold = appConfig.getDefaultIntValue("cycleThreshold", this.cycleThreshold);

        String filterName = appConfig.getProperty("datafilter");
        filter = DataFilterFactory.newInstance(filterName);
        filter.initialize(appConfig.getPeriodicDelay() * Constants.MS_TO_SEC);
        logger.infof(ILogger.TAG_EXECUTOR, "///// Instantiating data filter: %s", filter.getClass().getSimpleName());

    }

    /**
     * Device read cycle - executes a single time step of the automation system; loops through
     * each consumer and submits and then gets future
     *
     * @return  DataElementHolder
     */
    public DataElementHolder call()   {

        DataElementHolder holder = new DataElementHolder();

        // add an empty status message to holder
        holder.put(DataElementKey.STATUS_MESSAGE, new StringBufferDataElement(new StringBuffer()));

        if (xgvSpeedController == null)   {
            xgvSpeedController = instantiateXgvSpeedController();
        }

        // new futures list
        futures.clear();

        // submit tasks
        for (IConsumerTask consumer : consumers)   {
            Future<DataElementHolder> futureHolder = executor.submit(consumer);
            futures.add(futureHolder);
        }
        logger.debug(ILogger.TAG_EXECUTOR, "Futures tasks submitted.");

        // collect results
        for (int i=0; i<futures.size(); i++)   {
            Future<DataElementHolder> future = futures.get(i);
            //add all elements from this DataElementHolder
            try   {
                DataElementHolder collectedData = future.get();
                //logger.debug(ILogger.TAG_EXECUTOR, "**** This Consumer got: " + collectedData.toString());

                // any consumer can provide a STATUS_MESSAGE element.  However, we will add any value provided
                //  by the consumer to the primary executor element
                DataElement statusElement = collectedData.remove(DataElementKey.STATUS_MESSAGE);
                if (statusElement != null)   {
                    String tmpStatusMessage = ( (StringBufferDataElement) statusElement).value();
                    if (tmpStatusMessage != null && tmpStatusMessage.length() > 0)   {
                        holder.appendStatusMessage(tmpStatusMessage);
                    }
                }

                holder.putAll(collectedData);
            }
            catch(Exception e)   {
                // log.error
                logger.error(ILogger.TAG_EXECUTOR, "Exception getting future: " + e.getMessage());
            }
        }

        //package the ASD message data in a form that the Trajectory can use
        packageAsdMessages(holder);

        // ead also needs operating speed, so load into holder
        double operatingSpeedMph = DviParameters.getInstance().getOperatingSpeed();

        //TODO: when integrating into Carma this will need to be replaced by roadway speed limit or some other
        //      possibly variable desired speed
        DataElement operatingSpeed = new DoubleDataElement(ConversionUtils.getInstance().mphToMps(operatingSpeedMph));
        holder.put(DataElementKey.OPERATING_SPEED, operatingSpeed);

        double smoothedSpeed = getSmoothedSpeed(holder);

        // calculate motion status and add to holder
        DataElementHolder motionStatusHolder = computeMotionStatus(lastSpeed, smoothedSpeed);
        holder.putAll(motionStatusHolder);

        double acceleration = filter.getSmoothedDerivative();
        holder.put(DataElementKey.ACCELERATION, new DoubleDataElement(acceleration) );

        double jerk = filter.getSmoothedSecondDerivative();
        holder.put(DataElementKey.JERK, new DoubleDataElement(jerk));

        // compute target speed
        DataElementHolder eadHolder = null;
        try   {
            DateTime eadCommandStart = new DateTime();
            eadHolder = trajectory.getSpeedCommand(holder);

            Duration duration = new Duration(new DateTime(eadCommandStart), new DateTime());
            holder.put(DataElementKey.CYCLE_EAD, new IntDataElement((int) duration.getMillis()));

            holder.putAll(eadHolder);
        }
        catch(Exception e)   {
            //this is a serious problem that should never happen - shut down the system
            logger.error(ILogger.TAG_EXECUTOR, "Exception trapped from trajectory.getSpeedCommand. Aborting mission: "
                        + e.toString());
            GlidepathStateModel.getInstance().changeState(GlidepathState.FATAL, null);
        }

        boolean isValidated = false;

        // check whether override was EVER triggered once we were ENGAGED
        DataElement xgvStatusElement = holder.get(DataElementKey.XGV_STATUS);
        if (xgvStatusElement != null)   {
            XgvStatus xgvStatus = ((XgvStatusDataElement) xgvStatusElement).value();
            if (bOverrideEngaged)   {
                xgvStatus.setManualOverrideEngaged(true);
                holder.remove(DataElementKey.XGV_STATUS);
                holder.put(DataElementKey.XGV_STATUS, new XgvStatusDataElement(xgvStatus));
            }
        }
        // else, we are not communicating with the xgv, i.e. turned off XGV key
        else   {
            logger.error(ILogger.TAG_EXECUTOR, "Lost XGV communication! Shutting down!");
            GlidepathStateModel.getInstance().changeState(GlidepathState.FATAL, null);
        }

        // Direct XVG - state may have been changed by the Javascript code (e.g. user command to re-engage at red light)
        GlidepathState state = GlidepathStateModel.getInstance().getState();

        // we only direct if in ENGAGED state
        if ( state.equals(GlidepathState.ENGAGED) )   {
            if (!bEngaged)   {
                bEngaged = true;
                trajectory.engage();
            }

            // check to see if we stepped on brake or hit yellow button (xgvStatusElement guaranteed good at this point)
            XgvStatus xgvStatus = ((XgvStatusDataElement) xgvStatusElement).value();

            // this is true when brake or yellow button pressed
            if (xgvStatus.isManualOverrideEngaged())   {
                // set the flag so we know manual override has been triggered, even if in message not provided to
                // client
                bOverrideEngaged = true;

                // DISENGAGE eco drive
                GlidepathStateModel.getInstance().changeState(GlidepathState.DISENGAGED, null);
                bEngaged = false;
                logger.info(ILogger.TAG_EXECUTOR, "Manual override button pressed. Disengaging automation.");
            }

            isValidated = holder.validate();

            if (!isSpeedZero() )   {
                try   {
                    if (bEngaged)   {
                        double speedCommand = holder.getDoubleElement(DataElementKey.SPEED_COMMAND);
                        DateTime xgvCommandStart = new DateTime();

                        if (xgvSpeedController != null)   {
                            xgvSpeedController.sendMotionProfile(speedCommand);
                        }

                        Duration duration = new Duration(new DateTime(xgvCommandStart), new DateTime());
                        holder.put(DataElementKey.CYCLE_XGV_COMMAND, new IntDataElement((int) duration.getMillis()));
                    }
                }
                catch(Exception e)   {
                    logger.error(ILogger.TAG_EXECUTOR, "Error sendMotionProfile to XGV: " + e.getMessage());
                    e.printStackTrace();
                }
            }
            else   {
                // DISENGAGE eco drive
                GlidepathStateModel.getInstance().changeState(GlidepathState.DISENGAGED, null);
                bEngaged = false;
            }
        }

        // NOTE: This line is used to induce errors to test DVI in simulated mode
        //induceErrors(holder);

        // if we have a status message, add it to the queue
        String strStatus = ((StringBufferDataElement) holder.get(DataElementKey.STATUS_MESSAGE)).value();
        if (strStatus != null && strStatus.length() > 0)   {
            if (statusQueue.size() >= cycleThreshold)   {
                statusQueue.remove();
                statusQueue.add(strStatus);
            }
            else   {
                statusQueue.add(strStatus);
            }
        }

        // condition met to provide status message to DVI
        if (statusQueue.size() >= cycleThreshold && cycle < cycleMax)   {
            logger.warn(ILogger.TAG_EXECUTOR, getMessagesFromQueue(statusQueue).toString());
            resetQueues();
        }
        else if (cycle >= cycleMax)   {
            resetQueues();
        }
        else   {
            cycle += 1;
        }

        // finally, we are always providing the previousStatusQueue to the holder, which should display for cycleMax
        //   so we delete the current element and provide the contents of the previousStatusQueue
        holder.remove(DataElementKey.STATUS_MESSAGE);
        addStatusMessagesToHolder(holder);

        loop += 1;

        return holder;
    }

    /**
     * Determine if vehicle speed has settled to zero after a period of time (to make sure we're not just
     * seeing noise).
     *
     * @return true of speed hs confirmed to be zero after we have been in motion
     */
    private boolean isSpeedZero()   {
       return trajectory.isStopConfirmed();
    }


    private void resetQueues()   {
        if (statusQueue.size() < cycleThreshold)   {
            previousStatusQueue.clear();
        }
        else   {
            previousStatusQueue = statusQueue;
        }

        statusQueue = new LinkedList<String>();
        cycle = 0;
    }


    /**
     * Repackages all of the MAP & SPAT messages received from the ASD into a single list of intersections
     * that is usable by the Trajectory class.  All of this activity is done in the holder object whose
     * reference is passed in.
     * @param holder - data holder that contains all of the affected data elements
     */
    private void packageAsdMessages(DataElementHolder holder) {
        IntersectionCollection collection = new IntersectionCollection();

        //loop through all MAP messages in the holder
        List<IAsdMessage> mapList = ((IAsdListDataElement)holder.get(DataElementKey.MAP_LIST)).value();
        if (mapList != null) {
            for (IAsdMessage msg : mapList) {
                //get its intersections ID
                int id = msg.getIntersectionId();
                //search for this ID in the existing list
                int index = -1;
                if (collection.intersections != null  &&  collection.intersections.size() > 0) {
                    for (int i = 0; i < collection.intersections.size(); ++i) {
                        if (id ==  collection.intersections.get(i).intersectionId) {
                            index = i;
                            break;
                        }
                    }
                }
                //if the ID isn't already represented in the collection then
                if (index < 0) {
                    //add it
                    IntersectionData inter = new IntersectionData();
                    inter.intersectionId = id;
                    inter.map = (MapMessage)msg;
                    if (collection.intersections == null) {
                        collection.intersections = new ArrayList<IntersectionData>();
                    }
                    collection.intersections.add(inter);
                //else (we have info on this intersections already)
                }else {
                    //if this one's version is more recent than the MAP already present, replace it
                    int existingVersion = collection.intersections.get(index).map.getContentVersion();
                    MapMessage mapMsg = (MapMessage)msg;
                    int newVersion = mapMsg.getContentVersion();
                    if (newVersion > existingVersion  || (newVersion == 0  &&  existingVersion != 0)) { //allows for wrap-around
                        logger.debug(ILogger.TAG_EXECUTOR, "Ready to replace the existing mapMsg with new version.");
                        collection.intersections.get(index).map = mapMsg;
                    }
                }
            }
        }
        logger.debug(ILogger.TAG_EXECUTOR, "packageAsdMessages - map messages complete.");

        //loop through all SPAT messages in the holder
        List<IAsdMessage> spatList = ((IAsdListDataElement)holder.get(DataElementKey.SPAT_LIST)).value();
        if (spatList != null) {
            logger.debug(ILogger.TAG_EXECUTOR, "spatList contains " + spatList.size() + " items.");
            for (IAsdMessage msg : spatList) {
                //get its intersections ID
                int id = msg.getIntersectionId();
                //search for this ID in the existing list
                int index = -1;
                if (collection.intersections != null  &&  collection.intersections.size() > 0) {
                    for (int i = 0; i < collection.intersections.size(); ++i) {
                        if (id == collection.intersections.get(i).intersectionId) {
                            index = i;
                            break;
                        }
                    }
                }
                SpatMessage spatMsg = (SpatMessage)msg;

                //if the ID isn't already represented in the collection then
                if (index < 0) {
                    //add it
                    IntersectionData inter = new IntersectionData();
                    inter.intersectionId = id;
                    inter.spat = spatMsg;
                    if (collection.intersections == null) {
                        collection.intersections = new ArrayList<IntersectionData>();
                    }
                    collection.intersections.add(inter);
                }else {
                    //if this one's version number is larger (with wrapping), replace it
                    int existingVersion;
                    SpatMessage prevSpat = (SpatMessage)collection.intersections.get(index).spat;
                    if (prevSpat == null) {
                        existingVersion = -1;
                    }else {
                        existingVersion = prevSpat.getContentVersion();
                    }
                    int newVersion = spatMsg.getContentVersion();
                    if (newVersion > existingVersion  ||  (newVersion == 0  &&  existingVersion != 0)) {
                        collection.intersections.get(index).spat = spatMsg;
                    }
                }
            }
        }
        logger.debug(ILogger.TAG_EXECUTOR, "packageAsdMessages - spat messages complete.");

        //remove the MAP & SPAT lists from the data holder
        holder.remove(DataElementKey.MAP_LIST);
        holder.remove(DataElementKey.SPAT_LIST);

        //add the new intersections collection to the holder
        holder.put(DataElementKey.INTERSECTION_COLLECTION, new IntersectionCollectionDataElement(collection));
    }


    private String addStatusMessagesToHolder(DataElementHolder holder)   {
        StringBuffer sb = getMessagesFromQueue(previousStatusQueue);

        holder.put(DataElementKey.STATUS_MESSAGE, new StringBufferDataElement(sb));

        return sb.toString();
    }


    private StringBuffer getMessagesFromQueue(Queue<String> queue)   {
        StringBuffer sb = new StringBuffer();

        for (String value : queue)   {
            sb.append(value);
        }

        return sb;

    }


    /**
     * Adds current speed to filter and adds a SMOOTHED_SPEED element to the holder
     *
     * @param currentHolder
     * @return  double smoothed speed
     */
    private double getSmoothedSpeed(DataElementHolder currentHolder)   {

        double currentSpeed = currentHolder.getDoubleElement(DataElementKey.SPEED);

        filter.addRawDataPoint(currentSpeed);
        double smoothedSpeed = filter.getSmoothedValue();

        currentHolder.put(DataElementKey.SMOOTHED_SPEED, new DoubleDataElement(smoothedSpeed));

        return smoothedSpeed;
    }


    /**
     * Computes motion status based on lastSpeed and currentSpeed using a configured smoothing factor
     *
     * Last speed is computed as an average of the last X speeds, depending on the size of the configured queue
     * The smoothing factor is specified in meters per second
     *
     * @param lastSpeed double
     * @param currentSpeed double
     * @return
     */
    private DataElementHolder computeMotionStatus(double lastSpeed, double currentSpeed)   {
        MotionStatus motionStatus;

        String strSmoothing = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig()).getProperty("motion.status.smoothing");
        double motionStatusSmoothing = Double.parseDouble(strSmoothing);

        logger.debug("ComputeMotionStatus", " #### factor : lastSpeed : currentSpeed    ::: " + strSmoothing + " : " + lastSpeed + " : " + currentSpeed);

        if (lastSpeed == 0.0)   {
            motionStatus = MotionStatus.Stopped;
        }
        else if (currentSpeed > lastSpeed + motionStatusSmoothing)   {
            motionStatus = MotionStatus.Speeding_Up;
        }
        else if (currentSpeed < lastSpeed - motionStatusSmoothing)   {
            motionStatus = MotionStatus.Slowing_Down;
        }
        else   {
            motionStatus = MotionStatus.Coast;
        }

        DataElementHolder holder = new DataElementHolder();
        holder.put(DataElementKey.MOTION_STATUS, new MotionStatusDataElement(motionStatus));

        return holder;
    }


    /**
     * Stop the local executor and release thread resources
     */
    public void stop()   {
        // consumer termination handled above by DviExecutorService

        executor.shutdown();
    }


    /**
     * Set the list of consumers
     *
     * @param consumers
     */
    public void addConsumers(List<IConsumerTask> consumers)   {
        this.consumers = consumers;
    }

    /**
     * clear the consumer list
     */
    public void clearConsumers()   {
        consumers.clear();
    }

    /**
     * Instantiate an XgvSpeedController object
     *
     * @return  xgvSpeedController
     */
    private XgvSpeedController instantiateXgvSpeedController()   {
        XgvSpeedController xgvSpeedController = null;

        try   {
            for (IConsumerTask consumer : consumers)   {
                if (consumer instanceof XgvConsumer)   {
                    XgvConnection connection = ((XgvConsumer) consumer).getConnection();

                    xgvSpeedController = new XgvSpeedController(connection);
                    logger.debug("EXEC", "Instantiated XgvSpeedController.");
                }
            }
        }
        catch(Exception e)   {
            logger.error("EXEC", "Error instantiating an XgvSpeedController:" + e.getMessage());
        }

        return xgvSpeedController;
    }


    /**
     * Used as a mechanism to test error display for simulated DVI
     *
     * @param holder
     */
    private void induceErrors(DataElementHolder holder)   {
        if ( (loop > 100 && loop < 140) || loop > 200 && loop < 240)  {
            if (cycle == 2 || cycle ==4 || cycle == 18)   {
                holder.appendStatusMessage("Some status message for loop starting: " + loop);
            }
        }
    }
}
