package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

// Interface to any of the possible EAD algorithm classes that we may choose to use.

import gov.dot.fhwa.saxton.glidepath.asd.IntersectionData;
import gov.dot.fhwa.saxton.glidepath.ead.trajectorytree.ITreeSolver;

import java.util.List;

public interface IEad {

	/**
	 * Provides one-time initialization of the EAD algorithm object.
	 * @param timestep - duration of system time step, ms
	 * @param solver - the tree solver to use
	 */
	void initialize(long timestep, ITreeSolver solver);


	/**
	 * Calculates the new target speed to be used to command the vehicle's motion for the current time step.
	 * @param speed - current vehicle speed, m/s
	 * @param operSpeed - current desired speed, m/s
	 * @param accel - current vehicle forward acceleration, m/s^2
	 * @param intersections - list of known intersections, if any, sorted in order from nearest to farthest
	 * @return target speed, m/s
	 */
	double getTargetSpeed(double speed, double operSpeed, double accel,
						  List<IntersectionData> intersections) throws Exception;


	/**
	 * Allows specification of the longitudinal width of the intersections's stop box.
	 * @param width - width of the box, m
	 */
	void setStopBoxWidth(double width);


	/**
	 * Notifies the EAD object that the list of intersections has changed in some significant way
	 * since the previous time step.
	 */
	void intersectionListHasChanged();
	
}
