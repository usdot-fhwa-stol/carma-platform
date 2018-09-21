/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.signal_plugin.logger;

import java.io.IOException;

/**
 * Simple interface into native CARMA logging systems
 */
public class LoggerManager {

    /**
     * Create or retrieve an instance of an {@link ILogger} implementation for class origin.
     * @param origin The class for which the logger is requested
     * @return Either a preexisting logger instance spawned before or a new one
     */
    public static ILogger getLogger(Class origin) {
        return new CarmaLoggerProxy(gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager.getLogger(origin.getCanonicalName()));
    }

    public static void setRealTimeOutput(boolean val) {
        // NO-OP
    }

    public static void setOutputFile(String filename) {
        // NO-OP
    }

    /**
     * Gather up all the unwritten log entries from all Loggers produced from this factory. Collect the entries from the
     * uwritten buffer, merge them all into a collective buffer and write that buffer to file. Then notify each logger
     * that it's unwritten buffer has been saved to file.
     */
    public synchronized static void writeToDisk() throws IOException {
        // NO-OP
    }

    /**
     * Registers a callback with all currently instantiated ILoggers from this factory
     * @param listener The {@link LogListener} object to be registered
     */
    public static void registerGlobalLoggerCallback(LogListener listener) {
        // NO-OP
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
        // NO-OP
    }

    /**
     * Return value of current min setting of output
     *
     * @return  LogEntry.Level
     */
    public static LogEntry.Level getMinOutputToWrite() {
        return LogEntry.Level.DEBUG;
    }


    /**
     * Provide method to conditionally determine if we should add debug messages
     *
     * @return boolean true if DEBUG set
     */
    public static boolean isDebug() {
        return true;
    }


    /**
     * Close the logger for class origin, making it unable to write to file or other forms of output
     * @param origin The class to close the logger for
     */
    public static void closeLogger(Class origin) {
        // NO-OP
    }

    /**
     * Set whether we want consumer data logging to begin or stop
     *
     * @param flag
     */
    public static void setRecordData(boolean flag)   {
        // NO-OP
    }

    /**
     * Returns the state of the record data flag
     *
     * @return true to log, false to not log
     */
    public static boolean getRecordData()   {
        return true;
    }

    public static void useLog4J2(boolean val) {
        // NO-OP
    }

}
