package gov.dot.fhwa.saxton.glidepath.ead;

// basic 2D vector representation in Cartesian coordinates

public class CartesianVector2D {
	
	/**
	 * constructs a vector with base at point p1 and head at p2
	 * @param p1
	 * @param p2
	 */
	public CartesianVector2D(CartesianPoint2D p1, CartesianPoint2D p2) {
		
		//translate the vector so that its base is at the origin
		x_ = p2.x() - p1.x();
		y_ = p2.y() - p1.y();
	}
	
	/**
	 * constructs a vector with its origin at zero and the end point at the given coords
	 */
	public CartesianVector2D(double x, double y) {
		x_ = x;
		y_ = y;
	}
	
	/**
	 * @return x component of the vector
	 */
	public double x() {
		return x_;
	}
	
	/**
	 * @return y component of the vector
	 */
	public double y() {
		return y_;
	}
	
	/**
	 * always : a vector that is s times longer than this vector
	 */
	public CartesianVector2D multiply(double s) {
		CartesianVector2D res = new CartesianVector2D(s*x_, s*y_);
		return res;
	}
	
	/**
	 * always : dot product of v2 with this vector
	 */
	public double dotProduct(CartesianVector2D v2) {
		return x_*v2.x() + y_*v2.y();
	}
	
	/**
	 * always : magnitude of this vector
	 */
	public double magnitude() {
		return Math.sqrt(x_*x_ + y_*y_);
	}
	
	////////////////
	// class members
	////////////////
	
	private double			x_; //end point x coord
	private double			y_; //end point y coord
}
