package gov.dot.fhwa.saxton.carma.signal_plugin.filter;

/**
 * A quick & simple solver of matrix equations of the form AX = B,
 * where A is a 3x3 matrix of known values, X is a 3x1 vector of unknowns, and B is a 3x1 vector of known dependent values.
 * 
 * @author starkj
 *
 */
public class MatrixSolver {
	
	public MatrixSolver() {
		a_ = new double[3][3];
		b_ = new double[3];
		x_ = new double[3];
		for (int i = 0;  i < 3;  ++i) {
			for (int j = 0;  j < 3;  ++j) {
				a_[i][j] = 0.0;
			}
			b_[i] = 0.0;
			x_[i] = 0.0;
		}
	}
	
	public void setAElement(int i, int j, double val) {
		a_[i][j] = val;
	}
	
	public void setBElement(int i, double val) {
		b_[i] = val;
	}
	
	public double[] getResult() throws Exception {
		
		//perform the Gaussian elimination on A
		gauss();
		
		//perform the back-substitution to solve for the three values of X, in reverse order
		if (Math.abs(a_[2][2]) < 1.0e-10  ||  Math.abs(a_[1][1]) < 1.0e-10  ||  Math.abs(a_[0][0]) < 1.0e-10) {
			throw new Exception("Cannot back substitute to solve the matrix.");
		}
		x_[2] = b_[2] / a_[2][2];
		x_[1] = (b_[1] - x_[2]*a_[1][2]) / a_[1][1];
		x_[0] = (b_[0] - x_[2]*a_[0][2] - x_[1]*a_[0][1]) / a_[0][0];
		
		return x_;
	}

	///////////////////
	// private elements
	///////////////////
	
	/**
	 * This is a crude, brute force reduction; no time to think through an elegant, looping masterpiece.
	 * @throws Exception 
	 */
	private void gauss() throws Exception {
		
		//if element (1,1) is close to zero then
		if (Math.abs(a_[0][0]) < 1.0e-10) {
			int swapWith = 0;
			//find a row that isn't
			if (Math.abs(a_[1][0]) < 1.0e-10) {
				if (Math.abs(a_[2][0]) < 1.0e-10) {
					throw new Exception("Cannot perform Gaussian elimination on the matrix.");
				}
				swapWith = 2;
			}else {
				swapWith = 1;
			}
		
			//swap rows
			double temp;
			for (int j = 0;  j < 3;  ++j) {
				temp = a_[swapWith][j];
				a_[swapWith][j] = a_[0][j];
				a_[0][j] = temp;
			}
			temp = b_[swapWith];
			b_[swapWith] = b_[0];
			b_[0] = temp;
		}
		
		//eliminate the first column in rows 2 & 3
		for (int i = 1;  i < 3;  ++i) {
			double mult = a_[i][0] / a_[0][0];
			double adj;
			a_[i][0] = 0.0;
			for (int j = 1;  j < 3;  ++j) {
				adj = mult * a_[0][j];
				a_[i][j] -= adj;
			}
			adj = mult * b_[0];
			b_[i] -= adj;
		}
		
		//eliminate the 2nd column of row 3
		if (Math.abs(a_[1][1]) < 1.0e-10) {
			throw new Exception("Cannot perform Gaussian elimination on the matrix.");
		}
		double mult = a_[2][1] / a_[1][1];
		double adj = mult * a_[1][2];
		a_[2][1] = 0.0;
		a_[2][2] -= adj;
		adj = mult * b_[1];
		b_[2] -= adj;
	}
	
	private double[][]			a_;
	private double[]			b_;
	private double[]			x_;
}
