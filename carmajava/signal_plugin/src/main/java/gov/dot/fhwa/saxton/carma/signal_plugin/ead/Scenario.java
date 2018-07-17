package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

public enum Scenario {
	RAMP_UP,		//initial acceleration to operating speed at beginning of test run
	CONSTANT,		//no speed change required (scenario 1 in white paper)
	OVERSPEED,		//speed up to get through intersections before green expires (scenario 2)
	OVERSPEED_EXT,	//continuation of the overspeed state while vehicle passes through the stop box (scenario 2)
	SLOWING,		//slow down to wait for red to expire before we reach stop bar (scenario 4)
	GRADUAL_STOP,	//moving toward an eventual stop at stop bar and wait for red to expire (scenario 3)
	FINAL_STOP,		//final moments of the stopping maneuver (scenario 3)
	DEPARTURE		//egress from the intersections after crossing the stop bar
}
