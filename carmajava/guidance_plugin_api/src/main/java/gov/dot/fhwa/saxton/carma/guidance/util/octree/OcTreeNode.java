/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.util.octree;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.AxisAlignedBoundingBox;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.CartesianObject;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Structural tree node for {@link IntervalTree}
 * <p>
 * Holds a center value, and a set of values to the left and the right, uses strategy pattern objects
 * to determine how to calculate interval intersection and ordering as well as how to handle inserting
 * new datums into the tree.
 */
public class OcTreeNode<T> {

  protected SortedSet<OcTreeNode<T>> children;
  // TODO should the points within a node only exists at the lead or throughout?
  protected List<OcTreeDatum<T>> contents;

  protected CartesianObject region;
  AxisAlignedBoundingBox intersectionChecker = new AxisAlignedBoundingBox();

  OcTreeNode(CartesianObject region, List<OcTreeDatum<T>> contents) {
    this.region = region;
    this.contents = contents;
  }

  public boolean contains(Point3D p) {
    return intersectionChecker.intersects(region, p);
  }

  public void divide() {

    double[][] bb = region.getBounds();
    final int minI = CartesianObject.MIN_BOUND_IDX;
    final int maxI = CartesianObject.MAX_BOUND_IDX;

    // Child 1
    Point3D center = (Point3D) region.getCentroidOfBounds();
    Point3D point = new Point3D(bb[0][minI], bb[1][minI], bb[2][minI]);
    CartesianObject subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 2
    point = new Point3D(bb[0][minI], bb[1][minI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 3
    point = new Point3D(bb[0][minI], bb[1][maxI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 4
    point = new Point3D(bb[0][maxI], bb[1][minI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 5
    point = new Point3D(bb[0][minI], bb[1][maxI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 6
    point = new Point3D(bb[0][maxI], bb[1][minI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 7
    point = new Point3D(bb[0][maxI], bb[1][maxI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
    // Child 8
    point = new Point3D(bb[0][maxI], bb[1][maxI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new OcTreeNode<>(subRegion, contents));
  }


  @Override
  public String toString() {
    String dataString = "";
    for (Interval<?> interval : data) {
      dataString += interval.toString() + ",";
    }

    return String.format("IntervalTreeNode{center=%.02f,data=%s,left=%s,right=%s}", center, dataString, left,
        right);
  }
}