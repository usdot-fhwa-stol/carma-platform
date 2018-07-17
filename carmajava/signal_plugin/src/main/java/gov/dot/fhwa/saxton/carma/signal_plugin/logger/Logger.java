package gov.dot.fhwa.saxton.glidepath.logger;

/**
 * Created by rushk1 on 10/9/2014.
 */
public class Logger implements ILogger {
    private LogBuffer written = new LogBuffer();
    protected LogBuffer unwritten = new LogBuffer();
    private Class<Object> origin;
    private boolean realTimeOutput = false;

    public Logger(Class<Object> origin) {
        this.origin = origin;
    }

    @Override
    public LogEntry output(String tag, String message) {
        LogEntry entry = generateLogEntry(LogEntry.Level.DATA, tag, message, null);
        return log(entry);
    }

    @Override
    public LogEntry outputf(String tag, String message, Object... args) {
        LogEntry entry = generateLogEntry(LogEntry.Level.DATA, tag, String.format(message, args), null);
        return log(entry);
    }

    @Override
    public LogEntry debug(String tag, String message) {
        if (!LoggerManager.isDebug()) {
            return null;
        }
        LogEntry entry = generateLogEntry(LogEntry.Level.DEBUG, tag, message, null);
        return log(entry);
    }

    @Override
    public LogEntry error(String tag, String message) {
        LogEntry entry = generateLogEntry(LogEntry.Level.ERROR, tag, message, null);
        return log(entry);
    }

    @Override
    public LogEntry warn(String tag, String message) {
        LogEntry entry = generateLogEntry(LogEntry.Level.WARN, tag, message, null);
        return log(entry);
    }

    @Override
    public LogEntry info(String tag, String message) {
        LogEntry entry = generateLogEntry(LogEntry.Level.INFO, tag, message, null);
        return log(entry);
    }

    @Override
    public LogEntry debug(String tag, String message, Object extra) {
        if (!LoggerManager.isDebug()) {
            return null;
        }
        LogEntry entry = generateLogEntry(LogEntry.Level.DEBUG, tag, message, extra);
        return log(entry);
    }

    @Override
    public LogEntry error(String tag, String message, Object extra) {
        LogEntry entry = generateLogEntry(LogEntry.Level.ERROR, tag, message, extra);
        return log(entry);
    }

    @Override
    public LogEntry warn(String tag, String message, Object extra) {
        LogEntry entry = generateLogEntry(LogEntry.Level.WARN, tag, message, extra);
        return log(entry);
    }

    @Override
    public LogEntry info(String tag, String message, Object extra) {
        LogEntry entry = generateLogEntry(LogEntry.Level.INFO, tag, message, extra);
        return log(entry);
    }

    @Override
    public LogEntry debugf(String tag, String message, Object... args) {
        if (!LoggerManager.isDebug()) {
            return null;
        }
        LogEntry entry = generateLogEntry(LogEntry.Level.DEBUG, tag, String.format(message, args), null);
        return log(entry);
    }

    @Override
    public LogEntry warnf(String tag, String message, Object... args) {
        LogEntry entry = generateLogEntry(LogEntry.Level.WARN, tag, String.format(message, args), null);
        return log(entry);
    }

    @Override
    public LogEntry errorf(String tag, String message, Object... args) {
        LogEntry entry = generateLogEntry(LogEntry.Level.ERROR, tag, String.format(message, args), null);
        return log(entry);
    }

    @Override
    public LogEntry infof(String tag, String message, Object... args) {
        LogEntry entry = generateLogEntry(LogEntry.Level.INFO, tag, String.format(message, args), null);
        return log(entry);
    }

    /**
     * Generates and populates the fields of a new {@link LogEntry} object. Takes System.getCurrentTimeMillis
     * as the log entry's timestamp and this.origin as the log entry's origin. Not to be used outside the Logger class since
     * this method is primarily just to abstract out the common elements of each logging method.
     * @param level The severity/importance level of the message to be logged
     * @param tag The tag to be attached to the message to be logged
     * @param message The string content of the message to be logged
     * @param extra An extra argument to be used to store additional an additional object that could be retrieved later
     * @return The newly generated LogEntry object with information from the parameters, the logger object, and system time
     */
    private LogEntry generateLogEntry(LogEntry.Level level, String tag, String message, Object extra) {

        if (!LoggerManager.getRecordData())   {
            return null;
        }

        LogEntry log = new LogEntry(level, tag, message, System.currentTimeMillis(),
                extra, this.origin);

        if (realTimeOutput) {
            System.out.println(log.toString());
        }

        // Unsure if I should also have this method insert the LogEntry into the buffer. It would factor more code out of
        // the surrounding methods, but would make this method have side-effects when it doesn't have to.
        return log;
    }

    @Override
    public LogEntry log(LogEntry entry) {
        if (LoggerManager.getRecordData())   {
            unwritten.insert(entry);
        }
        // Eventually this is where the hooks for LoggerCallbacks will go
        return entry;
    }

    @Override
    public LogEntry throwExcept(String tag, String message, Exception e) {
        LogEntry entry = generateLogEntry(LogEntry.Level.ERROR, tag, message, e);
        return entry;
    }

    @Override
    public LogEntry caughtExcept(String tag, String message, Exception e) {
        LogEntry entry = generateLogEntry(LogEntry.Level.ERROR, tag, message, e);
        return entry;
    }

    @Override
    public void notifyWritten() {
        // Merge the contents of both buffers then empty the unwritten buffer
        written = LogBuffer.merge(written, unwritten);
        unwritten = new LogBuffer();
    }

    protected LogBuffer getBuffer() {
        return this.unwritten;
    }

    @Override
    public void setRealTimeOutput(boolean realTimeOutput) {
        this.realTimeOutput = realTimeOutput;
    }

    @Override
    public void registerLogListener(LogListener listener) {
        throw new UnsupportedOperationException();
    }
}
