package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

public class ObsoleteDataException extends Exception {
	
	public ObsoleteDataException() {
		super();
	}
	
	public ObsoleteDataException(String message) {
		super(message);
	}

	private static final long serialVersionUID = 698293819943404499L;
}
