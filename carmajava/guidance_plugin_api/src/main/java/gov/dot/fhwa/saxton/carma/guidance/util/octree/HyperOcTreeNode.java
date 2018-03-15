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
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
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
public class HyperOcTreeNode<T> {

  private static final int NUM_SUB_REGIONS_PER_DIM[] = {0, 2, 4, 8, 16, 32, 64, 128, 256};

  protected SortedSet<HyperOcTreeNode<T>> children;
  protected List<HyperOcTreeDatum<T>> contents;

  protected CartesianObject region;
  AxisAlignedBoundingBox intersectionChecker = new AxisAlignedBoundingBox();

  HyperOcTreeNode(CartesianObject region, List<HyperOcTreeDatum<T>> contents) {
    this.region = region;
    this.contents = contents;
  }

  public boolean contains(Point p) {
    return intersectionChecker.intersects(region, p);
  }

  public void divide() {

    double[][] bb = region.getBounds();
    final int minI = CartesianObject.MIN_BOUND_IDX;
    final int maxI = CartesianObject.MAX_BOUND_IDX;

    int numDim = region.getNumDimensions();
    int numSubRegions;

    if (numDim >= NUM_SUB_REGIONS_PER_DIM.length) {
      numSubRegions = (int)Math.pow(2, numDim);
    } else {
      numSubRegions = NUM_SUB_REGIONS_PER_DIM[numDim];
    }

    // TODO think about if there is a way to cache all possible point combinations ahead of time
    // That way we avoid the check for min/max on each divide call
    Point center = region.getCentroidOfBounds();
    int[] minMax = {minI, maxI};
    for (int i = 0; i< numSubRegions; i++) {
      
      double[] pointData = new double[numDim];
      for (int j = 0; j < numDim; j++) {
        pointData[j] = bb[j][minMax[(i & ( 1 << j)) != 0 ? 1 : 0]];
      }
      Point point = new Point(pointData);
      CartesianObject subRegion = new CartesianObject(Arrays.asList(point, center));
      children.add(new HyperOcTreeNode<>(subRegion, contents));
    }
  }


  public boolean insert(HyperOcTreeDatum<T> datum, HyperOcTreeConditions<T> conditions) {
    return insert(datum, conditions, new DiscardCollector<T>());
  }

  public boolean insert(HyperOcTreeDatum<T> datum, HyperOcTreeConditions<T> conditions,
    HyperOcTreeCollector<T> collector) {

    if (!contains(datum.getPoint())) {
      collector.collect(this, datum, false);
      return false;
    }
    // If this the datum triggers an invalid insert from the conditions return false without inserting
    if (!conditions.validInsert(this, datum)) {
      return false;
    }
    // Check if this is the lowest we need to go to insert the datum
    if (conditions.doneInsert(this, datum)) {
      contents.add(datum);
      collector.collect(this, datum, true);
      return true;
    }
    // If this node has no children create them
    if (children.isEmpty()) { 
      // No data stored in this node currently
      divide();
      // Move datum to new respective child node
      for (HyperOcTreeDatum<T> content: contents) {
        for (HyperOcTreeNode<T> child: children) {
          if (child.insert(content, conditions)) {
            break;
          }
        }
      }
    }

    // Continue to insert new datum. Only one child will actually have the datum inserted
    for (HyperOcTreeNode<T> child: children) {
      if (child.insert(datum, conditions)) {
        collector.collect(this, datum, true);
        return true;
      }
    }
    // If we go through all children without an insert then this datum cannot be inserted
    collector.collect(this, datum, false);
    return false;
  }


  @Override
  public String toString() {
    String dataString = "";
    for (HyperOcTreeDatum<?> datum : contents) {
      dataString += datum.toString();
    }

    return String.format("OcTreeTreeNode{contents: " + dataString + ", region: " + region + "}");
  }
}

/*

    // Child 1
    Point center = region.getCentroidOfBounds();
    Point point = new Point(bb[0][minI], bb[1][minI], bb[2][minI]);
    CartesianObject subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 2
    point = new Point(bb[0][minI], bb[1][minI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 3
    point = new Point(bb[0][minI], bb[1][maxI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 4
    point = new Point(bb[0][maxI], bb[1][minI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 5
    point = new Point(bb[0][minI], bb[1][maxI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 6
    point = new Point(bb[0][maxI], bb[1][minI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 7
    point = new Point(bb[0][maxI], bb[1][maxI], bb[2][minI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
    // Child 8
    point = new Point(bb[0][maxI], bb[1][maxI], bb[2][maxI]);
    subRegion = new CartesianObject(Arrays.asList(point, center));
    children.add(new HyperOcTreeNode<>(subRegion, contents));
  */