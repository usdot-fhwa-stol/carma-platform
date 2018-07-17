package gov.dot.fhwa.saxton.carma.signal_plugin.filter;

import gov.dot.fhwa.saxton.glidepath.filter.IDataFilter;

/**
 * This class is a simple substitution for accessing the raw speed data stream directly, 
 * but allows the architecture to use the filtering approach at all times.
 * 
 * @author starkj
 *
 */
public class NoFilter implements IDataFilter {
	
	@Override
	public void initialize(double timeStep) {
		current_ = 0.0;
		prev1_ = 0.0;
		prev2_ = 0.0;
		timeStep_ = timeStep;
	}

	@Override
	public void addRawDataPoint(double rawValue) {
		//need to store current point plus two historical points
		prev2_ = prev1_;
		prev1_ = current_;
		current_ = rawValue;
	}

	@Override
	public double getSmoothedValue() {
		return current_;
	}

	@Override
	public double getSmoothedDerivative() {
		return (current_ - prev1_) / timeStep_;
	}

	@Override
	public double getSmoothedSecondDerivative() {
		double curDeriv = (current_ - prev1_) / timeStep_;
		double prevDeriv = (prev1_ - prev2_) / timeStep_;
		double secondDeriv = (curDeriv - prevDeriv) / timeStep_;
		return secondDeriv;
	}

	//////////////////
	// member elements
	//////////////////
	
	private double				current_;
	private double				prev1_;
	private double				prev2_;
	private double				timeStep_;
}
