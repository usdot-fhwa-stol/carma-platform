package gov.dot.fhwa.saxton.carma.signal_plugin.logger;

import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;

/**
 * Created by rushk1 on 10/9/2014.
 */
public class LogEntry {
    public static enum Level {
        DATA,
        ERROR,
        WARN,
        INFO,
        DEBUG
    }

    public Level getLevel() {
        return level;
    }

    public String getTag() {
        return tag;
    }

    public String getMessage() {
        return message;
    }

    public Long getTimestamp() {
        return timestamp;
    }

    public Object getContents() {
        return contents;
    }

    public Class<Object> getOrigin() {
        return origin;
    }

    public String getNotes() {
        return notes;
    }

    private Level level;

    public LogEntry(Level level, String tag, String message, Long timestamp, Object contents, Class<Object> origin) {
        this.level = level;
        this.tag = tag;
        this.message = message;
        this.timestamp = timestamp;
        this.contents = contents;
        this.origin = origin;
    }

    private String tag, message;
    private Long timestamp;
    private Object contents;
    private Class<Object> origin;
    protected String notes;


    @Override
    /**
     * This method is not only to be used for debugging and console output purposes, but also controls the output format
     * of the log message when written to file. This behavior may change in the future if necessary.
     */
    public String toString() {
        DateTime dt = new DateTime(timestamp);
        DateTimeFormatter fmt = DateTimeFormat.forPattern("HH:mm:ss.SSS");
        return fmt.print(dt)  + "\t" + padAndTruncate(level.toString(), 6) + "\t" +
                padAndTruncate(origin.getSimpleName(), 25) + "\t" +
                padAndTruncate(tag, 6) + "\t" + message;
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
}
