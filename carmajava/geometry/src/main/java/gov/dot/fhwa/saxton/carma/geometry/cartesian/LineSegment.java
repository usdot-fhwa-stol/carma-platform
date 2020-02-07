/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * A representation of a line segment in N-dimensional space.
 */
public class LineSegment implements CartesianElement {
  protected Point p1;
  protected Point p2;
  protected Vector vector;

  /**
   * Constructor defines a line segment as the straight path between two provided points
   * @param p1 The first point
   * @param p2 The second point. Must be of the same dimension as the first point
   * @throws IllegalArgumentException Thrown if p1 and p2 are not the same dimension
   */
  public LineSegment(Point p1, Point p2) throws IllegalArgumentException{
    if (p1.getNumDimensions() != p2.getNumDimensions()) {
      throw new IllegalArgumentException("Point dimensions do not match");
    }
    this.p1 = p1;
    this.p2 = p2;
    this.vector = new Vector(p1,p2);
  }

  /**
   * Constructor defines this line as a deep copy of the provided line segment
   * @param seg The segment to be copied
   */
  public LineSegment(LineSegment seg) {
    this.p1 = new Point(seg.getP1());
    this.p2 = new Point(seg.getP2());
    this.vector = new Vector(this.p1, this.p2);
  }

  /**
   * Returns the length of the line
   * @return line length
   */
  public double length() {
    return vector.magnitude();
  }

  @Override public int getNumDimensions(){
    return p1.getNumDimensions();
  }

  /**
   * Returns the perpendicularDistance between this line and the provided point
   * @param point The point to compare
   * @return The perpendicular distance to the point
   * @throws IllegalArgumentException Thrown if this line and the provided point are not of the same dimension
   */
  public double perpendicularDistance(Point point) throws IllegalArgumentException {
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // Equation giving shortest distance to a line in n-dimensional space
    // x = a + tn defines a line
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

  /**
   * Returns the point which lies on the infinite line defined by this line segment
   * @param point The point to project
   * @return The point resulting from projection
   * @throws IllegalArgumentException Thrown if the provided point does not have the same dimensions as this line segment
   */
  public Point pointProjectedOnLine(Point point) throws IllegalArgumentException{
    if (this.getNumDimensions() != point.getNumDimensions()) {
      throw new IllegalArgumentException("dimensions do not match");
    }
    // Based off of same math as in perpendicularDistance
    Vector a = new Vector(this.p1);
    Vector p = new Vector(point);
    Vector n = this.vector.getUnitVector();

    Vector a_p = a.subtract(p);
    Vector a_p_n = n.scalarMultiply(a_p.dot(n));

    return (a.add(a_p_n.scalarMultiply(-1.0))).toPoint();
  }

  /**
   * Gets the first point of this line segment
   * @return the first point
   */
  public Point getP1() {
    return p1;
  }

  /**
   * Gets the second point of this line segment
   * @return The second point
   */
  public Point getP2() {
    return p2;
  }

  /**
   * Gets the vector formed by this line segment running from p1 to p2
   * @return The vector
   */
  public Vector getVector() {
    return vector;
  }
}
