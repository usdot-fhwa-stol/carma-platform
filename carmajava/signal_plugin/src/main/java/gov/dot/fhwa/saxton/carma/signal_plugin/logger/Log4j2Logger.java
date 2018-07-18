package gov.dot.fhwa.saxton.carma.signal_plugin.logger;

import org.apache.logging.log4j.*;

/**
 * ILogger wrapper around Log4J2 logger
 */
public class Log4j2Logger implements ILogger {

    public static final int TAG_LENGTH = 6;
    private org.apache.logging.log4j.Logger log;
    private Class origin;

    public Log4j2Logger(Class origin) {
        log = LogManager.getLogger(origin);
        this.origin = origin;
    }

    private static final String ROUTING_KEY = "ROUTING_KEY";
    private static final String DATA_LEVEL_ID = "DATA";
    private static final int DATA_LEVEL = 100;

    @Override
    public LogEntry output(String tag, String message) {
        log.log(Level.forName(DATA_LEVEL_ID,  DATA_LEVEL), padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.DATA, tag, message, null);
    }

    @Override
    public LogEntry outputf(String tag, String message, Object... args) {
       return output(tag, String.format(message, args));
    }

    @Override
    public LogEntry debug(String tag, String message) {
        log.debug(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.DEBUG, tag, message, null);
    }

    @Override
    public LogEntry error(String tag, String message) {
        log.error(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.ERROR, tag, message, null);
    }

    @Override
    public LogEntry warn(String tag, String message) {
        log.warn(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.WARN, tag, message, null);
    }

    @Override
    public LogEntry info(String tag, String message) {
        log.info(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.INFO, tag, message, null);
    }

    @Override
    public LogEntry debug(String tag, String message, Object extra) {
        log.debug(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.DEBUG, tag, message, null);
    }

    @Override
    public LogEntry error(String tag, String message, Object extra) {
        log.error(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.ERROR, tag, message, extra);
    }

    @Override
    public LogEntry warn(String tag, String message, Object extra) {
        log.warn(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.WARN, tag, message, extra);
    }

    @Override
    public LogEntry info(String tag, String message, Object extra) {
        log.info(padAndTruncate(tag, TAG_LENGTH) + "\t" + message);
        return generateLogEntry(LogEntry.Level.INFO, tag, message, extra);
    }

    @Override
    public LogEntry debugf(String tag, String message, Object... args) {
        log.debug(padAndTruncate(tag, TAG_LENGTH) + "\t" + String.format(message, args));
        return generateLogEntry(LogEntry.Level.DEBUG, tag, message, null);
    }

    @Override
    public LogEntry warnf(String tag, String message, Object... args) {
        log.warn(padAndTruncate(tag, TAG_LENGTH) + "\t" + String.format(message, args));
        return generateLogEntry(LogEntry.Level.WARN, tag, message, null);
    }

    @Override
    public LogEntry errorf(String tag, String message, Object... args) {
        log.error(padAndTruncate(tag, TAG_LENGTH) + "\t" + String.format(message, args));
        return generateLogEntry(LogEntry.Level.ERROR, tag, message, null);
    }

    @Override
    public LogEntry infof(String tag, String message, Object... args) {
        log.info(padAndTruncate(tag, TAG_LENGTH) + "\t" + String.format(message, args));
        return generateLogEntry(LogEntry.Level.INFO, tag, message, null);
    }

    @Override
    public LogEntry log(LogEntry entry) {
        switch (entry.getLevel()) {
            case INFO:
                info(entry.getTag(), entry.getMessage());
                break;
            case DEBUG:
                debug(entry.getTag(), entry.getMessage());
                break;
            case WARN:
                warn(entry.getTag(), entry.getMessage());
                break;
            case ERROR:
                error(entry.getTag(), entry.getMessage());
                break;
            case DATA:
                output(entry.getTag(), entry.getMessage());
                break;
        }
        return entry;
    }

    @Override
    public LogEntry throwExcept(String tag, String message, Exception e) {
        return error(tag, message);
    }

    @Override
    public LogEntry caughtExcept(String tag, String message, Exception e) {
        return error(tag, message);
    }

    @Override
    public void notifyWritten() {
        // noop
    }

    @Override
    public void registerLogListener(LogListener listener) {
        // noop
    }

    @Override
    public void setRealTimeOutput(boolean val) {
        // noop
    }

    private LogEntry generateLogEntry(LogEntry.Level level, String tag, String message, Object extra) {
        LogEntry log = new LogEntry(level, tag, message, System.currentTimeMillis(),
                extra, this.origin);
        return log;
    }

    /**
     * Formats a string to a fixed length, padding it with spaces if it's too short or truncating it with ... if it's
     * too long.
     * @param in The string to be processed
     * @param length The fixed length the input string must adhere to
     * @return The processed string trucated or padded as needed
     */
    private String padAndTruncate(String in, int length) {
        if (in.length() < length) {
            // Right-pad the string to meet the length requirement
            return String.format("%1$-" + length + "s", in);
        } else if (in.length() > length) {
            return in.substring(0, length - 3) + "...";
        } else {
            return in;
        }
    }

    protected void setLogRoute(String routeName) {
        ThreadContext.put(ROUTING_KEY, routeName);
    }
}
