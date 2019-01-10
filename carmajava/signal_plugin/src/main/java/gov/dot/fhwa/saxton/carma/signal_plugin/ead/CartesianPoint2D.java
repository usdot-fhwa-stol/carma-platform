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

// represents a point in a Cartesian coordinate system in 2D space

public class CartesianPoint2D {
	
	/**
	 * defines a point in 2D space with input coords in that Cartesian system
	 */
	public CartesianPoint2D(double x, double y) {
		x_ = x;
		y_ = y;
	}
	
	/**
	 * @return x coordinate of the point
	 */
	public double x() {
		return x_;
	}
	
	/**
	 * @return y coordinate of the point
	 */
	public double y() {
		return y_;
	}
	
	/**
	 * always : the distance between p2 and this point
	 */
	public double distanceFrom(CartesianPoint2D p2) {
		double dx = p2.x() - x_;
		double dy = p2.y() - y_;
		return Math.sqrt(dx*dx + dy*dy);
	}
	
	/**
	 * always : a point that is translated from this point's location by the vector t
	 */
	public CartesianPoint2D translate(CartesianVector2D t) {
		CartesianPoint2D res = new CartesianPoint2D(x_ + t.x(), y_ + t.y());
		return res;
	}
	
	//////////////////
	// member elements
	//////////////////
	
	private double				x_;
	private double				y_;
}
