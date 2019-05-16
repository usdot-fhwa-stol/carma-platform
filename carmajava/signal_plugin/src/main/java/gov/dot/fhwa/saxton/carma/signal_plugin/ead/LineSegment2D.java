/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

// Represents a finite line segment between two adjacent points in a Cartesian coordinate system in 2D.
// Calculations are based on http://geomalgorithms.com/a02-_lines.html

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.CartesianPoint2D;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.CartesianVector2D;

public class LineSegment2D {
	
	/**
	 * p1 != p2 : creates a line segment from two end points
	 * p1 == p2 : exception
	 */
	public LineSegment2D(CartesianPoint2D p1, CartesianPoint2D p2) throws Exception {
		
		if (p1.x() == p2.x()  &&  p1.y() == p2.y()) {
			throw new Exception("Attempting to create a LineSegment2D with two identical points.");
		}

		//store a vector representing the segment as well as the end points
		p0_ = p1;
		p1_ = p2;
		v_ = new CartesianVector2D(p1, p2);
	}
	
	/**
	 * perpendicular from p to the segment is between the segment end points : length of the perpendicular
	 * perpendicular from p to the segment falls outside the end points : -1
	 * 
	 * @param p - the point in question
	 * @return shortest distance between p and the line segment
	 */
	public double shortestDistanceToPoint(CartesianPoint2D p) {
		double result = 0.0;
		
		//define a vector, w, from the tail of the segment to the given point
		CartesianVector2D w = new CartesianVector2D(p0_, p);
		
		//get the dot product of w with v
		double dotProd = v_.dotProduct(w);
		
		//if this dot product is negative or > |v| then set result to a negative value (p is not between the end points of the segment)
		if (dotProd < 0.0) {
			result = -1.0;
		}else {
			double magV  = v_.magnitude();
			double magV2 = magV * magV;
			if (dotProd > magV2) {
				result = -1.0;
			//else (p is between the segment endpoints)
			}else {
				//compute the perpendicular distance
				double b = dotProd / magV2; //safe since this segment is guaranteed to have two distinct points, so magV > 0 always
				CartesianPoint2D pb = p0_.translate(v_.multiply(b));
				result = pb.distanceFrom(p);
			}
		}
		
		return result;
	}
	
	/**
	 * always : shortest distance from p to the infinite line that contains this segment
	 * 
	 * @param p - the point in question
	 * @return - distance
	 */
	public double shortestDistanceToPointExtendedSegment(CartesianPoint2D p) {
		
		//define a vector, w, from the head of the segment to the given point
		CartesianVector2D w = new CartesianVector2D(p0_, p);
		
		//get the dot product of w with v
		double dotProd = v_.dotProduct(w);
		
		//compute the perpendicular distance
		double magV = v_.magnitude();
		double magV2 = magV * magV;
		double b = dotProd / magV2; //safe since this segment is guaranteed to have two distinct points, so mag > 0 always
		CartesianPoint2D pb = p0_.translate(v_.multiply(b));
		double result = pb.distanceFrom(p);
		
		return result;
	}
	
	/**
	 * always : the length of the line segment
	 */
	public double length() {
		return p0_.distanceFrom(p1_);
	}
	
	/**
	 * perpendicular from p to the line segment is within the segment endpoints : point where the perpendicular meets the segment
	 * else : null
	 */
	public CartesianPoint2D translateTo(CartesianPoint2D p) {
		CartesianPoint2D result = null;
		
		//define a vector, w, from the tail of the segment to the given point
		CartesianVector2D w = new CartesianVector2D(p0_, p);
		
		//get the dot product of w with v
		double dotProd = v_.dotProduct(w);
		
		//if this dot product is non-negative and <= |v| then p is between the endpoints, so calc a result
		if (dotProd >= 0.0) {
			double magV  = v_.magnitude();
			double magV2 = magV * magV;
			if (dotProd <= magV2) {
				//compute the perpendicular distance
				double b = dotProd / magV2; //safe since this segment is guaranteed to have two distinct points, so magV > 0 always
				//find the point at the base of the perpendicular
				result = p0_.translate(v_.multiply(b));
			}
		}
		
		return result;
	}
	
	public CartesianPoint2D translateToExtendedSegment(CartesianPoint2D p) {
		CartesianPoint2D result = null;
		
		//define a vector, w, from the head of the segment to the given point
		CartesianVector2D w = new CartesianVector2D(p0_, p);
		
		//get the dot product of w with v
		double dotProd = v_.dotProduct(w);
		
		//compute the perpendicular
		double magV = v_.magnitude();
		double magV2 = magV * magV;
		double b = dotProd / magV2; //safe since this segment is guaranteed to have two distinct points, so mag > 0 always
		//get the point at the base of the perpendicular
		result = p0_.translate(v_.multiply(b));

		return result;
	}
	
	////////////////
	// class members
	////////////////
	
	private CartesianPoint2D			p0_;
	private CartesianPoint2D			p1_;
	private CartesianVector2D			v_;
}
