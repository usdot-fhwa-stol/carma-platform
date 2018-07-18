package gov.dot.fhwa.saxton.carma.signal_plugin.logger;


/**
 * Thin logger proxy for adapting the Glidepath ILogger to the Carma ILogger
 */
public class CarmaLoggerProxy implements ILogger {
	private gov.dot.fhwa.saxton.carma.guidance.util.ILogger logger;
	private Class origin;

    public CarmaLoggerProxy(gov.dot.fhwa.saxton.carma.guidance.util.ILogger logger) {
        this.logger = logger;
	}

    private LogEntry generateLogEntry(LogEntry.Level level, String tag, String message, Object extra) {
        LogEntry log = new LogEntry(level, tag, message, System.currentTimeMillis(),
                extra, this.origin);
        return log;
    }

	@Override
	public LogEntry output(String tag, String message) {
		logger.info(tag, message);
		return generateLogEntry(LogEntry.Level.DATA, tag, message, null);
	}

	@Override
	public LogEntry outputf(String tag, String message, Object... args) {
		logger.infof(tag, message, args);
		return generateLogEntry(LogEntry.Level.DATA, tag, String.format(message, args), null);
	}

	@Override
	public LogEntry debug(String tag, String message) {
		logger.debug(tag, message);
		return generateLogEntry(LogEntry.Level.DEBUG, tag, message, null);
	}

	@Override
	public LogEntry error(String tag, String message) {
		logger.error(tag, message);
		return generateLogEntry(LogEntry.Level.ERROR, tag, message, null);
	}

	@Override
	public LogEntry warn(String tag, String message) {
		logger.warn(tag, message);
		return generateLogEntry(LogEntry.Level.WARN, tag, message, null);
	}

	@Override
	public LogEntry info(String tag, String message) {
		logger.info(tag, message);
		return generateLogEntry(LogEntry.Level.INFO, tag, message, null);
	}

	@Override
	public LogEntry debug(String tag, String message, Object extra) {
		logger.debug(tag, message);
		return generateLogEntry(LogEntry.Level.DEBUG, tag, message, extra);
	}

	@Override
	public LogEntry error(String tag, String message, Object extra) {
		logger.error(tag, message);
		return generateLogEntry(LogEntry.Level.ERROR, tag, message, extra);
	}

	@Override
	public LogEntry warn(String tag, String message, Object extra) {
		logger.warn(tag, message);
		return generateLogEntry(LogEntry.Level.WARN, tag, message, extra);
	}

	@Override
	public LogEntry info(String tag, String message, Object extra) {
		logger.info(tag, message);
		return generateLogEntry(LogEntry.Level.INFO, tag, message, info);
	}

	@Override
	public LogEntry debugf(String tag, String message, Object... args) {
		logger.debugf(tag, message, args);
		return generateLogEntry(LogEntry.Level.DEBUG, tag, String.format(message, args), null);
	}

	@Override
	public LogEntry warnf(String tag, String message, Object... args) {
		logger.warnf(tag, message, args);
		return generateLogEntry(LogEntry.Level.WARN, tag, String.format(message, args), null);
	}

	@Override
	public LogEntry errorf(String tag, String message, Object... args) {
		logger.errorf(tag, message, args);
		return generateLogEntry(LogEntry.Level.ERROR, tag, String.format(message, args), null);
	}

	@Override
	public LogEntry infof(String tag, String message, Object... args) {
		logger.infof(tag, message, args);
		return generateLogEntry(LogEntry.Level.INFO, tag, String.format(message, args), null);
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
		return error(tag, message, e);
	}

	@Override
	public LogEntry caughtExcept(String tag, String message, Exception e) {
		return warn(tag, message, e);
	}

	@Override
	public void notifyWritten() {
		// NO-OP
	}

	@Override
	public void registerLogListener(LogListener listener) {
		// NO-OP	
	}

	@Override
	public void setRealTimeOutput(boolean val) {
		// NO-OP
	}
}