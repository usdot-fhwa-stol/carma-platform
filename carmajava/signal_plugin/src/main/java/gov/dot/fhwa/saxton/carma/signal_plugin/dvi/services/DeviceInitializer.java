package gov.dot.fhwa.saxton.glidepath.dvi.services;


import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * DeviceInitializer
 *
 * Submits initializer for each device in the consumers list
 *
 */
public class DeviceInitializer {

    ILogger logger = LoggerManager.getLogger(DeviceInitializer.class);

    /**
     * list of consumers
     */
    List<IConsumerTask> consumers = new ArrayList<IConsumerTask>();

    /**
     * Executor service to submit consumers
     */
    ExecutorService executor = Executors.newFixedThreadPool(5);

    /**
     * list of futures for each consumer task
     */
    List<Future<Boolean>> futuresInit = new ArrayList<Future<Boolean>>();

    /**
     * default constructor
     */
    public DeviceInitializer()   {
        // intentionally left blank
    }

    /**
     * Constructor with provided list of consumers
     *
     * @param consumers
     */
    public DeviceInitializer(List<IConsumerTask> consumers)   {
        this.consumers = consumers;
    }

    /**
     * Add a consumer to the list
     * @param consumer
     */
    public void addConsumer(IConsumerTask consumer)   {
        consumers.add(consumer);
    }

    /**
     * Clear the consumer list
     */
    public void clearConsumers()   {
        consumers.clear();
    }

    /**
     * Run the initializers for each consumer in the list
     *
     * Returns false if ANY initializer fails
     *
     * @return boolean indicating success or failure
     */
    public boolean start()   {

        // make sure we have consumers
        if (consumers.isEmpty())   {
            return false;
        }

        boolean result = true;

        // iterate the consumers, initialize them, acquire Initializer and submit
        for (IConsumerTask consumer : consumers)   {
            try   {
            	logger.debugf(ILogger.TAG_EXECUTOR, "consumer initialization loop: consumer = %s", consumer.getClass().getSimpleName());
                consumer.initialize();
                IConsumerInitializer initializer = consumer.getInitializer();
                Future<Boolean> initFuture = executor.submit(initializer);
                futuresInit.add(initFuture);
            }
            catch(Exception e)   {
                logger.error(ILogger.TAG_EXECUTOR, "Exception thrown while submitting initializer: " + e.getMessage());
                return false;
            }
        }

        // iterate and check futures and return false if ANY fail
        for (int i=0; i<futuresInit.size(); i++)   {
            Future<Boolean> initFuture = futuresInit.get(i);

            try   {
                if (!initFuture.get())   {
                    IConsumerTask consumer = consumers.get(i);
                    logger.error("INIT", "Initialization of consumer has failed. All consumers are being terminated: " + consumer.getClass().getSimpleName());
                    terminateAllConsumers();
                    return false;
                }
            }
            catch(Exception e)   {
                logger.error(ILogger.TAG_EXECUTOR, "Exception thrown while checking future: " + e.getMessage());
                IConsumerTask consumer = consumers.get(i);
                logger.error("INIT", "Initialization of consumer has failed. All consumers are being terminated: " + consumer.getClass().getSimpleName());
                terminateAllConsumers();
                return false;
            }
        }

        try { LoggerManager.writeToDisk(); } catch(IOException ioe) { }

        return result;
    }

    /**
     * terminate all consumers
     *
     * if a consumer fails to initialize, we terminate all of them
     */
    private void terminateAllConsumers()   {
        if (consumers != null)   {
            for (IConsumerTask consumer : consumers)   {
                consumer.terminate();
            }
        }
    }
}
