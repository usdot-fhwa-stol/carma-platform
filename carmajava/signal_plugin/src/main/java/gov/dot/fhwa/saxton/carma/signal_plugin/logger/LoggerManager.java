package gov.dot.fhwa.saxton.carma.signal_plugin.logger;

import gov.dot.fhwa.saxton.glidepath.appcommon.GlidepathVersion;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.core.LoggerContext;
import org.apache.logging.log4j.core.config.Configuration;
import org.apache.logging.log4j.core.config.LoggerConfig;
import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Created by rushk1 on 10/9/2014.
 */
public class LoggerManager {
    private static HashMap<Class, ILogger> loggerInstances = new HashMap<>();
    private static String outputFilename = "log.txt";
    private static ArrayList<LogListener> globalListeners = new ArrayList<>();
    private static LogEntry.Level minOutputToWrite = null;
    private static boolean realTimeOutput = false;
    private static boolean firstWrite = true;
    private static boolean useLog4J2 = true;
    private static AtomicBoolean bRecordData = new AtomicBoolean(true);        // true, normal log operation...false, gobble and ignore

    /**
     * Create or retrieve an instance of an {@link ILogger} implementation for class origin.
     * @param origin The class for which the logger is requested
     * @return Either a preexisting logger instance spawned before or a new one
     */
    public static ILogger getLogger(Class origin) {
        // Search to see if we already have an active logger instance for that class, make a new one if not
        if (loggerInstances.containsKey(origin)) {
            return loggerInstances.get(origin);
        } else {
            ILogger logger;
            if (useLog4J2) {
                logger = new Log4j2Logger(origin);
            } else {
                logger = new Logger(origin);
            }

            // Register all current listeners with the new logger
            for (LogListener listener : globalListeners) {
                logger.registerLogListener(listener);
            }

            logger.setRealTimeOutput(realTimeOutput);

            loggerInstances.put(origin, logger);
            return logger;
        }
    }

    public static void setRealTimeOutput(boolean val) {
        for (ILogger log : loggerInstances.values()) {
            log.setRealTimeOutput(val);
        }
        realTimeOutput = val;
    }

    public static void setOutputFile(String filename) {
        outputFilename = filename;
        if (useLog4J2) {
            for (ILogger logger : loggerInstances.values()) {
                if (logger instanceof Log4j2Logger) {
                    ((Log4j2Logger) logger).setLogRoute(filename);
                }
            }
        }
    }

    /**
     * Gather up all the unwritten log entries from all Loggers produced from this factory. Collect the entries from the
     * uwritten buffer, merge them all into a collective buffer and write that buffer to file. Then notify each logger
     * that it's unwritten buffer has been saved to file.
     */
    public synchronized static void writeToDisk() throws IOException {
        if (useLog4J2) {
            // No need, Log4J2 handles its own writing
            return;
        }

        // Collect all the LogEntries by non-destructively merging them
        LogBuffer toBeWritten = new LogBuffer();
        for (ILogger logger : loggerInstances.values()) {
            toBeWritten = toBeWritten.merge(toBeWritten, ((Logger) logger).getBuffer());
        }

        // Optionally filter the data based on desired output level
        if (minOutputToWrite != null) {
            toBeWritten = toBeWritten.filterByLogLevel(minOutputToWrite);
        }

        // Open the FileOutputStream in append=true mode
        FileOutputStream outstream = new FileOutputStream(new File(outputFilename), true);

        // Write the date to the top of the file
        if (firstWrite) {
            PrintStream out = new PrintStream(outstream);
            DateTime dt = DateTime.now();
            DateTimeFormatter fmt = DateTimeFormat.forPattern("MM-dd-yyyy");
            String version = new GlidepathVersion().getFullId();
            out.println(fmt.print(dt) + "    " + version);

            firstWrite = false;
        }

        toBeWritten.write(outstream);
        outstream.close();

        // Notify each Logger of the write
        for (ILogger logger : loggerInstances.values()) {
            logger.notifyWritten();
        }
    }

    /**
     * Registers a callback with all currently instantiated ILoggers from this factory
     * @param listener The {@link LogListener} object to be registered
     */
    public static void registerGlobalLoggerCallback(LogListener listener) {
        for (ILogger logger : loggerInstances.values()) {
            logger.registerLogListener(listener);
        }
    }

    /**
     * Only LogEntries more severe than min will be written to output file. Severity is defined in {@link LogEntry.Level}.
     * Low ordinal values in the Level enum are considered higher severity.
     *
     * By default no filtering is performed so all log levels will be outputted and no performance cost will be incurred.
     * After this method is invoked however, ever call thereafter to writeToDisk will have to iterate the output
     * {@link LogBuffer} to filter {@link LogEntry}s based on
     * their severity level even if the minimum output to write is set to the lowest log level again.
     *
     * @param min The minimum log severity to be written, inclusive.
     */
    public static void setMinOutputToWrite(LogEntry.Level min) {
        minOutputToWrite = min;

        if (useLog4J2) {
            LoggerContext ctx = (LoggerContext) LogManager.getContext(false);
            Configuration config = ctx.getConfiguration();
            LoggerConfig loggerConfig = config.getLoggerConfig(LogManager.ROOT_LOGGER_NAME);

            Level log4j2Level = Level.DEBUG;
            switch (min) {
                case DATA:
                    log4j2Level = Level.forName("DATA", 100);
                    break;
                case INFO:
                    log4j2Level = Level.INFO;
                    break;
                case DEBUG:
                    log4j2Level = Level.DEBUG;
                    break;
                case WARN:
                    log4j2Level = Level.WARN;
                    break;
                case ERROR:
                    log4j2Level = Level.ERROR;
                    break;
            }

            loggerConfig.setLevel(log4j2Level);
            ctx.updateLoggers();
        }
    }

    /**
     * Return value of current min setting of output
     *
     * @return  LogEntry.Level
     */
    public static LogEntry.Level getMinOutputToWrite() {
        return minOutputToWrite;
    }


    /**
     * Provide method to conditionally determine if we should add debug messages
     *
     * @return boolean true if DEBUG set
     */
    public static boolean isDebug() {
        return (minOutputToWrite == LogEntry.Level.DEBUG);
    }


    /**
     * Close the logger for class origin, making it unable to write to file or other forms of output
     * @param origin The class to close the logger for
     */
    public static void closeLogger(Class origin) {
        throw new UnsupportedOperationException();
    }

    /**
     * Set whether we want consumer data logging to begin or stop
     *
     * @param flag
     */
    public static void setRecordData(boolean flag)   {
        bRecordData.getAndSet(flag);
    }

    /**
     * Returns the state of the record data flag
     *
     * @return true to log, false to not log
     */
    public static boolean getRecordData()   {
        return bRecordData.get();
    }

    public static void useLog4J2(boolean val) {
        useLog4J2 = val;
    }

}
