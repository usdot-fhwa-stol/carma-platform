package gov.dot.fhwa.saxton.carma.signal_plugin.logger;

import gov.dot.fhwa.saxton.glidepath.logger.LogEntry;

/**
 * Created by rushk1 on 10/9/2014.
 */
public interface LogListener {
    /**
     * Takes in a {@link LogEntry} that has been flagged as relevant by LogListener#checkEntry.
     * @param in The relevant LogEntry
     */
    public void newLogEntryCallback(LogEntry in);
}
