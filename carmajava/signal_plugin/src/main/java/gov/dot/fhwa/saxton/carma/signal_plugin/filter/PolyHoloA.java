package gov.dot.fhwa.saxton.carma.signal_plugin.filter;

import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

/**
 * Fits a polynomial of order 2 to smooth the raw data, using 15 historical data points.
 * Uses Holoborodko filters for first and second derivatives, the first derivative using
 * 15 historical points, and the second derivative using 11 historical points.  The
 * Holoborodko method is used courtesy of Pavel Holoborodko (http://www.holoborodko.com/pavel).
 * 
 * @author starkj
 *
 */
public class PolyHoloA implements IDataFilter {
	
	public PolyHoloA() {
		polyPoints_ = 15; //treat this like a constant, but can be revised if done prior to calling initialize()
		timeStep_ = 0.8888888;
	}

	@Override
	public void initialize(double timeStep) {
		timeStep_ = timeStep;
		arraySize_ = Math.max(polyPoints_, HOLO_MAX_PTS);
		raw_ = new double[arraySize_];
		coefA_ = 0.0;
		coefB_ = 0.0;
		coefC_ = 0.0;
		
		//since the X values are are time since the beginning of the historical buffer, we can pre-compute these sums
		sumX_ = 0.0;
		sumX2_ = 0.0;
		sumX3_ = 0.0;
		sumX4_ = 0.0;
		double time;
		double time2;
		for (int i = 0;  i < polyPoints_;  ++i) {
			time = timeStep * (double)i;
			time2 = time*time;
			sumX_ += time;
			sumX2_ += time2;
			sumX3_ += time*time2;
			sumX4_ += time2*time2;
		}
		
		failedPoint_ = false;
	}

	@Override
	public void addRawDataPoint(double rawValue) {
		
		//add the new value to the array
		updateArray(rawValue);
		++numPoints_;
		failedPoint_ = false;

		//Note: we are using the polynomial notation that y = a + b*x + c*x^2
		
		//if the array of historical data is full then
		if (numPoints_ >= polyPoints_) {

			//initialize the coefficients in case we can't solve the matrix (divide by zero)
			coefA_ = 0.0;
			coefB_ = 0.0;
			coefC_ = 0.0;
			
			//compute the sums necessary for the matrix elements; X represents time since the beginning of the array
			// at a constant time step separation, Y (the array values) represents the raw speed value at that time.
			// Our convention for time will be that 0 equates to the oldest data point and increments SIZE times to
			// the most recent data point.  Since our look-back distance is always the same number of points, all of
			// the sums involving only X will always be the same, so they are pre-calculated. We only need
			// to compute the sums involving Y.
			double sumY = 0.0;
			double sumXY = 0.0;
			double sumX2Y = 0.0;
			
			int offset = arraySize_ - polyPoints_; //polyPoints_ may be less than array size; we only want the most recent points
			for (int i = 0;  i < polyPoints_;  ++i) {
				sumY += raw_[i + offset];
				double time = timeStep_ * (double)i;
				sumXY += time * raw_[i + offset];
				sumX2Y += time*time * raw_[i + offset];
			}
			
			//set up the matrix equation and solve it
			MatrixSolver m = new MatrixSolver();
			m.setAElement(0, 0, polyPoints_);
			m.setAElement(0, 1, sumX_);
			m.setAElement(0, 2, sumX2_);
			
			m.setAElement(1, 0, sumX_);
			m.setAElement(1, 1, sumX2_);
			m.setAElement(1, 2, sumX3_);
			
			m.setAElement(2, 0, sumX2_);
			m.setAElement(2, 1, sumX3_);
			m.setAElement(2, 2, sumX4_);
			
			m.setBElement(0, sumY);
			m.setBElement(1, sumXY);
			m.setBElement(2, sumX2Y);
			
			double[] coef = new double[3];
			try {
				coef = m.getResult();
			} catch (Exception e) {
				//indicate this data point can't be analyzed
				failedPoint_ = true;
			}
			coefA_ = coef[0];
			coefB_ = coef[1];
			coefC_ = coef[2];
			
		} //endif have a full array
		
	}

	@Override
	public double getSmoothedValue() {
		double x = 0.0;
		double y = 0.0;
		
		//if we are dealing with a failed data point solution then
		if (failedPoint_) {
			//extrapolate from the 1st and 3rd raw points
			double slope = (raw_[arraySize_-1] - raw_[arraySize_-3]) / (2.0*timeStep_);
			y = raw_[polyPoints_-1] + timeStep_*slope;
			log_.debug("FILT", "Failed smoothing point. Interpolating.");
		
		//else
		}else {
			//solve the polynomial at the current value
			x = timeStep_ * (double)(polyPoints_-1);
			y = coefA_ + x*(coefB_ + x*coefC_);
		}
		double noise = raw_[arraySize_ - 1] - y;
		log_.infof("FILN", "getSmoothedValue noise =\t%.4f", noise);

		return y;
	}

	@Override
	public double getSmoothedDerivative() {
		double sum = 0.0;
		final int i = 15; //for ease of reading since we are storing newest data at highest index
		int offset = arraySize_ - i - 1;
		
		sum += 322.0*raw_[i-0 + offset];
		sum += 217.0*raw_[i-1 + offset];
		sum += 110.0*raw_[i-2 + offset];
		sum +=  35.0*raw_[i-3 + offset];
		sum -=  42.0*raw_[i-4 + offset];
		sum -=  87.0*raw_[i-5 + offset];
		sum -= 134.0*raw_[i-6 + offset];
		sum -= 149.0*raw_[i-7 + offset];
		sum -= 166.0*raw_[i-8 + offset];
		sum -= 151.0*raw_[i-9 + offset];
		sum -= 138.0*raw_[i-10 + offset];
		sum -=  93.0*raw_[i-11 + offset];
		sum -=  50.0*raw_[i-12 + offset];
		sum +=  25.0*raw_[i-13 + offset];
		sum +=  98.0*raw_[i-14 + offset];
		sum += 203.0*raw_[i-15 + offset];
		
		double result = sum/2856.0/timeStep_;
		
		return result;
	}

	@Override
	public double getSmoothedSecondDerivative() {
		double sum = 0.0;
		final int n = 11; //for ease of reading since we are storing newest data at highest index
		final int m = 5;
		int offset = arraySize_ - n - 1;
		
		sum -= 28.0*raw_[n-m + offset];
		sum -= 14.0*(raw_[n-m-1 + offset] + raw_[n-m+1 + offset]);
		sum +=  8.0*(raw_[n-m-2 + offset] + raw_[n-m+2 + offset]);
		sum += 13.0*(raw_[n-m-3 + offset] + raw_[n-m+3 + offset]);
		sum +=  6.0*(raw_[n-m-4 + offset] + raw_[n-m+4 + offset]);
		sum +=      (raw_[n-m-5 + offset] + raw_[n-m+5 + offset]);
		
		double result = sum / (256.0*timeStep_*timeStep_);
		
		return result;
	}
	
	/**
	 * newSize >= 3  &&  initialize() has not yet been called : resets the number of historical points used for polynomial
	 * else : no action
	 * 
	 * CAUTION: this method is intended for unit testing - use it only if you are very familiar with the consequences!
	 */
	public void setAlternatePolyPoints(int newSize) {
		//if input is reasonable and we have not already initialized the object then
		if (newSize >= 3  &&  Math.abs(timeStep_ - 0.8888) < 0.0002) {
			//specify an alternate size
			polyPoints_ = newSize;
			log_.warnf("FILT", "///// CAUTION: Filter is now using an alternate number of polynomial points: %d", newSize);
		}
	}

	//////////////////
	// member elements
	//////////////////
	
	/**
	 * always : shifts array contents to the left by one cell (toward lower indexes) and overwrites the highest index
	 * 			with the input value (data is newer as index increases)
	 */
	private void updateArray(double value) {
		for (int i = 0;  i < arraySize_-1;  ++i) {
			raw_[i] = raw_[i+1];
		}
		raw_[arraySize_-1] = value;
	}
	
	private double				timeStep_;
	private double[]			raw_;			//array of raw data, most recent is at [SIZE-1], oldest is at [0]
	private int					numPoints_;		//number of data points stored so far
	private double				coefA_;			//constant coefficient
	private double				coefB_;			//coefficient for x
	private double				coefC_;			//coefficient for x^2
	private double				sumX_;			//sum of X values
	private double				sumX2_;			//sum of X^2 values
	private double				sumX3_;			//sum of X^3 values
	private double				sumX4_;			//sum of X^4 values
	private boolean				failedPoint_;	//did the most recent data point fail a solution (e.g. matrix divide by zero)?
	private int					polyPoints_;	//total number of historical data points we will look at for the polynomial curve fit
	private int					arraySize_;		//number of historical points stored
	
	private static final int	HOLO_MAX_PTS = 16;	//max number of data points needed for Holoborodko filters
	private static ILogger log_ = LoggerManager.getLogger(PolyHoloA.class);
}
