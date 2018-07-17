package gov.dot.fhwa.saxton.glidepath.appcommon;

public enum SignalPhase {
	GREEN(0),
	YELLOW(1),
	RED(2),
    NONE(3);            // this value is used to indicate missing data

	SignalPhase(int val) {
		this.val = val;
	}
	
	public int value() {
		return this.val;
	}
	
	public SignalPhase next() {
		return values()[(ordinal()+1) % values().length];
	}
	
	private int val;
}
