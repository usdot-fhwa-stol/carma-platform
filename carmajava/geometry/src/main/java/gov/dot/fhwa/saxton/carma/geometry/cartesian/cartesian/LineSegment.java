/*
 * Copyright (C) 2017 Michael McConnell.
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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.cartesian;// Change

/**
 * A representation of a point in N-dimensional space.
 */
public class LineSegment implements DimensionalObject {
  private Point p1;
  private Point p2;
  private Vector vector;

  public LineSegment(Point p1, Point p2) throws IllegalArgumentException{
    if (p1.getNumDimensions() != p2.getNumDimensions()) {
      throw new IllegalArgumentException("Point dimensions do not match");
    }
    this.p1 = p1;
    this.p2 = p2;
    this.vector = new Vector(p1,p2);
  }

  public double length() {
    return vector.magnitude();
  }

  public int getNumDimensions(){
    return p1.getNumDimensions();
  }

  public double distanceToPointExtendedSegment(Point point) throws IllegalArgumentException {
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // Equation giving shortest distance to a line in n-dimensional space
    // x = a +tn defines a line 
    // where a is a point on the line. (As a vector)
    // n is the unit vector of the line.
    // t is a scalar of the unit vector.
    // x is the locus of the line
    // Then
    // P is the point (as a vector with the same root as a)
    // distance = (a - p) - ((a-p)*n)n
    
    if (this.getNumDimensions() != point.getNumDimensions()) {
      throw new IllegalArgumentException("dimensions do not match");
    }

    Vector a = new Vector(this.p1);
    Vector p = new Vector(point);
    Vector n = this.vector.getUnitVector();

    Vector a_p = a.subtract(p);
    Vector a_p_n = n.scalarMultiply(a_p.dot(n));

    return ((a_p).subtract(a_p_n)).magnitude();
  }

  //TODO this is wrong just like in the haversine calculation
  public double distanceToPoint(Point point){
    double extendedDistance = distanceToPointExtendedSegment(point);
    double distanceP1 = p1.distanceFrom(point);
    double distanceP2 = p2.distanceFrom(point);

    return Math.min(extendedDistance, Math.min(distanceP1, distanceP2));
    
  }

  //Was refered to as translateTo in old code
  public Point pointProjectedOnLine(Point point){
    // Based off of same math as in distanceToPointExtendedSegment
    Vector a = new Vector(this.p1);
    Vector p = new Vector(point);
    Vector n = this.vector.getUnitVector();

    Vector a_p = a.subtract(p);
    Vector a_p_n = n.scalarMultiply(a_p.dot(n));

    return (a.add(a_p_n)).toPoint();
  }
}
