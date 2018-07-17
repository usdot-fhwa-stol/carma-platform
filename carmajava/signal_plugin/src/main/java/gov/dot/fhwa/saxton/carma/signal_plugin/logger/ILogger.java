package gov.dot.fhwa.saxton.glidepath.logger;

/**
 * Created by rushk1 on 10/9/2014.
 */
public interface ILogger {

    public final static String TAG_GPS = "GPS";
    public final static String TAG_EXECUTOR = "EXEC";
    public final static String TAG_DVI = "DVI";

    /**
     * Log a message at {@link LogEntry.Level#DATA} level.
     *
     * The DATA log level is for data required to be logged by the formal requirements specification.
     *
     * @param tag A string representing the category of the data
     * @param message A string containing the message to be logged
     * @return A reference to the generate log entry
     */
    public LogEntry output(String tag, String message);

    /**
     * Log a message at {@link LogEntry.Level#DATA} level. Formats the message much
     * like printf() in C using the Object[] as the format string arguments.
     *
     * The DATA log level is for data required to be logged by the formal requirements specification.
     *
     * @param tag A string representing the category of the data
     * @param message A string containing the message to be logged, with format argument placeholders
     * @param args Zero or more arguments to fill the format placeholders
     * @return A reference to the generate log entry.
     */
    public LogEntry outputf(String tag, String message, Object... args);

    /**
     * Log a message at {@link LogEntry.Level#DEBUG} level.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @return A reference to the generated log entry
     */
    public LogEntry debug(String tag, String message);

    /**
     * Log a message at {@link LogEntry.Level#ERROR} level.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @return A reference to the generated log entry
     */
    public LogEntry error(String tag, String message);

    /**
     * Log a message at {@link LogEntry.Level#WARN} level.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @return A reference to the generated log entry
     */
    public LogEntry warn(String tag, String message);

    /**
     * Log a message at {@link LogEntry.Level#ERROR} level.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @return A reference to the generated log entry
     */
    public LogEntry info(String tag, String message);

    /**
     * Log a message at the {@link LogEntry.Level#DEBUG} level with an attached argument Object. The Object will be
     * included in both the text version of the log and in the LogEntry object as {@link LogEntry#contents}.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param extra An Object to be recorded in the logs and in the LogEntry data
     * @return A reference to the generated log entry
     */
    public LogEntry debug(String tag, String message, Object extra);

    /**
     * Log a message at the {@link LogEntry.Level#ERROR} level with an attached argument Object. The Object will be
     * included in both the text version of the log and in the LogEntry object as {@link LogEntry#contents}.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param extra An Object to be recorded in the logs and in the LogEntry data
     * @return A reference to the generated log entry
     */
    public LogEntry error(String tag, String message, Object extra);

    /**
     * Log a message at the {@link LogEntry.Level#WARN} level with an attached argument Object. The Object will be
     * included in both the text version of the log and in the LogEntry object as {@link LogEntry#contents}.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param extra An Object to be recorded in the logs and in the LogEntry data
     * @return A reference to the generated log entry
     */
    public LogEntry warn(String tag, String message, Object extra);

    /**
     * Log a message at the {@link LogEntry.Level#INFO} level with an attached argument Object. The Object will be
     * included in both the text version of the log and in the LogEntry object as {@link LogEntry#contents}.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param extra An Object to be recorded in the logs and in the LogEntry data
     * @return A reference to the generated log entry
     */
    public LogEntry info(String tag, String message, Object extra);

    /**
     * Log a message at the {@link LogEntry.Level#DEBUG} level with formatted messaged. Follows format
     * like C printf() using f_args[] as the format arguments. Any unmatched arguments/placeholders will be discarded
     * and replaced with ''
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param args An indefinite number of Object arguments representing the items to be matched to their placeholders
     * @return A reference to the generated log entry
     */
    public LogEntry debugf(String tag, String message, Object... args);

    /**
     * Log a message at the {@link LogEntry.Level#WARN} level with formatted messaged. Follows format
     * like C printf() using f_args[] as the format arguments. Any unmatched arguments/placeholders will be discarded
     * and replaced with ''
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param args An indefinite number of Object arguments representing the items to be matched to their placeholders
     * @return A reference to the generated log entry
     */
    public LogEntry warnf(String tag, String message, Object... args);

    /**
     * Log a message at the {@link LogEntry.Level#ERROR} level with formatted messaged. Follows format
     * like C printf() using f_args[] as the format arguments. Any unmatched arguments/placeholders will be discarded
     * and replaced with ''
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param args An indefinite number of Object arguments representing the items to be matched to their placeholders
     * @return A reference to the generated log entry
     */
    public LogEntry errorf(String tag, String message, Object... args);

    /**
     * Log a message at the {@link LogEntry.Level#INFO} level with formatted messaged. Follows format
     * like C printf() using f_args[] as the format arguments. Any unmatched arguments/placeholders will be discarded
     * and replaced with ''
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param args An indefinite number of Object arguments representing the items to be matched to their placeholders
     * @return A reference to the generated log entry
     */
    public LogEntry infof(String tag, String message, Object... args);

    /**
     * Insert a custom LogEntry object into the log queue
     * @param entry The log object to be inserted
     */
    public LogEntry log(LogEntry entry);

    /**
     * Throws an exception and generates a log entry containing information about it. The exception will be included
     * in the text version of the log entry and stored as an object in the log entry itself.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param e The exception to be thrown and recorded
     * @return A reference to the generated log entry
     */
    public LogEntry throwExcept(String tag, String message, Exception e);

    /**
     * Use to record an exception that was caught. Similar to {@link #throwExcept(String, String, Exception)}, but it
     * does not handle the actual exception. Use primarily in catch{} blocks to log handled exceptions.
     * @param tag A string representing the category of the data
     * @param message A string containing a message to be logged
     * @param e The exception being caught and recorded
     * @return
     */

    public LogEntry caughtExcept(String tag, String message, Exception e);

    /**
     * Notifies the logger that it's contents have been written to disk by the {@link LoggerManager}.
     * This should cause it to mark all presently unwritten/unsaved LogEntries as saved/written.
     */
    public void notifyWritten();

    /**
     * Registers a LogListener implementation to receive new data as it arrives at the Logger implementation
     * @param listener An object implementing the LogListener interface that
     */
    public void registerLogListener(LogListener listener);

    /**
     * Sets the ILogger to print output to STDOUT as it generates new log entries or not. The default state for an
     * ILogger is silent.
     * @param val If true, print to STDOUT. If false remain silent.
     */
    public void setRealTimeOutput(boolean val);
}
