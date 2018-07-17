package gov.dot.fhwa.saxton.glidepath.logger;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * Created by rushk1 on 10/9/2014.
 */

// Thin wrapper to ArrayList<LogEntry>.
// Not entirely sure if this is necessary, but leaving room for future functionality if needed. Present goal is to
// ensure that the list is always sorted when accessed.
//
// Can easily refactor out if this ends up being unnecessary and adds additional complexity
// - Kyle

public class LogBuffer implements Iterable {
    private ArrayList<LogEntry> localBuffer = new ArrayList<LogEntry>();

    public LogBuffer() {

    }

    /**
     * Adds an entry to the buffer while insuring that the log entries remain in ascending timestamp sorted order
     * @param entry The log entry to add to the list
     */
    public synchronized void insert(LogEntry entry) {
        int i = 0;
        // Walk the current list of logs until we find one that comes after it
        // If there are multiple entries with the same timestamp, we add the new one at the end of that block
        for (LogEntry log : localBuffer) {
            if (log.getTimestamp().compareTo(entry.getTimestamp()) <= 0) {
                i++;
            } else {
                break;
            }
        }
        localBuffer.add(i, entry);
    }

    public int getSize() {
        return localBuffer.size();
    }

    public LogEntry getLog(int index) {
        return localBuffer.get(index);
    }

    /**
     * Non-destructively merge buffers log1 and log2. Does not guarantee stability of sort order of same-timestamp
     * entries.
     * @param log1 A buffer to be merged
     * @param log2 A buffer to be merged
     * @return A new buffer object containing the sorted data from both log1 and log2
     */
    public static LogBuffer merge(final LogBuffer log1, final LogBuffer log2) {
        LogBuffer out = new LogBuffer();

        // Walk both buffers simultaneously and merge them together
        int i = 0, j = 0;
        LogEntry e1;
        LogEntry e2;
        for (; i < log1.getSize() && j < log2.getSize();) {
            e1 = log1.getLog(i);
            e2 = log2.getLog(j);
            if (e1.getTimestamp().compareTo(e2.getTimestamp()) <= 0) {
                out.localBuffer.add(e1);
                i++;
            } else {
                out.localBuffer.add(e2);
                j++;
            }
        }

        // Finish copying the tail of the array that is longer
        if (i < log1.getSize()) {
            out.localBuffer.addAll(log1.localBuffer.subList(i, log1.getSize()));
        } else if (j < log2.getSize()) {
            out.localBuffer.addAll(log2.localBuffer.subList(j, log2.getSize()));
        }

        return out;
    }

    @Override
    public Iterator iterator() {
        // Anonymous inner class with access to the current LogBuffer object
        return new Iterator() {
            private int current = 0;

            @Override
            public boolean hasNext() {
                return current < LogBuffer.this.getSize();
            }

            @Override
            public Object next() {
                return LogBuffer.this.getLog(current++);
            }

            @Override
            public void remove() {
                // Really don't want to be using this
                throw new UnsupportedOperationException();
            }
        };
    }

    /**
     * Walks the array of LogEntries associated with this LogBuffer and writes their ASCII representation
     * @param outputStream The OutputStream to write the ASCII data to
     */
    public void write(OutputStream outputStream) {
        PrintStream out = new PrintStream(outputStream);
        for (LogEntry entry : localBuffer) {
            out.println(entry.toString());
        }
        out.flush();
    }

    /**
     * Generates a new LogBuffer object that contains all the LogEntries in this LogBuffer except those below the input
     * severity level.
     * @param lowest The lowest level of severity which will be copied into the new LogBuffer
     * @return A LogBuffer containing the filtered LogEntries
     */
    public LogBuffer filterByLogLevel(LogEntry.Level lowest) {
        LogBuffer out = new LogBuffer();
        for (LogEntry e : localBuffer) {
            if (e.getLevel().ordinal() <= lowest.ordinal()) {
                out.insert(e);
            }
        }
        return out;
    }
}
