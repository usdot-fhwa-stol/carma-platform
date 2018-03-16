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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spacialhashmap;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import java.util.LinkedList;
import java.util.List;

/**
 * An implementation of an OcTree for use in the Conflict Detector
 * 
 * Used to evaluate trajectories and mobility paths for conflicts
 */
public class ConflictDetectorOcTree<T> {
  // protected HyperOcTreeNode<T> root; // TODO need to define the root and allow for changes in dimensions
  // // Axis number of route values in a point on the tree
  // private static final int DOWNTRACK_IDX = 0;
  // private static final int CROSSTRACK_IDX = 1;
  // private static final int TIME_IDX = 2;
  // // The maximum size of a cell in the tree
  // private double[] maxSize = {5,5,0.1};
  
  // // Conditions used for different insertion methods
  // HyperOcTreeConditions<T> noConflictCondition = new NoConflictConditions<>(maxSize);
  // HyperOcTreeConditions<T> sizeConditions = new MaxSizeConditions<>(maxSize);
  // HyperOcTreeConditions<T> noInsertConditions = new NoInsertConditions<>(maxSize);


  // /**
  //  * Constructor
  //  */
  // protected ConflictDetectorOcTree() {}


  // /**
  //  * Helper function which attempts to insert a list of points into the tree using the provided conditions.
  //  * If the allOrNothing flag is set, a failure to insert a single point will result in no points being added
  //  * 
  //  * @param points The list of downtrack, crosstrack, time points to add
  //  * @param conditions HyperOcTreeConditions which defines insertion behavior of the tree
  //  * @param allOrNothing If true, a failure to insert a single point will result in no points being added
  //  * 
  //  * @return A list of detected conflicts between the provided points and the current tree contents
  //  */
  // private List<ConflictSpace> insertWithConditions(List<Point3D> points, HyperOcTreeConditions<T> conditions, boolean allOrNothing) {

  //   List<HyperOcTreeDatum<T>> insertedData = new LinkedList<>();
  //   List<ConflictSpace> conflicts = new LinkedList<>();
  //   ConflictSpace currentConflict = null;
  //   int lane = 0; // TODO lane for each point
  //   Point prevPoint = null;
  //   for (Point p: points) {
  //     HyperOcTreeDatum<T> datum = new HyperOcTreeDatum<T>(p, null);// TODO maybe we should do the insert with the lane information as the datum
  //     // Failure to insert the point will mean a conflict has occurred
  //     if (!root.insert(datum, noConflictCondition)) { 
  //       // If no conflict is being tracked this is a new conflict
  //       if (currentConflict == null) {
  //         currentConflict = new ConflictSpace(p.getDim(DOWNTRACK_IDX), p.getDim(TIME_IDX), lane);
  //       } else if (lane != currentConflict.getLane()) {
  //         // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
  //         currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
  //         currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
  //         conflicts.add(currentConflict);
  //         // Use the current point's lane but the previous points distance and time to define the start of the new conflict
  //         currentConflict = new ConflictSpace(prevPoint.getDim(DOWNTRACK_IDX), prevPoint.getDim(TIME_IDX), lane);
  //       } 
  //     } else {
  //       insertedData.add(datum); // Add the inserted point to the list of inserted points
  //       // If we could insert the point but we are tracking a conflict then that conflict is done
  //       if (currentConflict != null) {
  //         currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
  //         currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
  //         conflicts.add(currentConflict);
  //         currentConflict = null; // Stop tracking the conflict
  //       }
  //     }
  //     prevPoint = p;
  //   }
  //   // Close the currentConflict if it was extending past the last point
  //   if (currentConflict != null) {
  //     currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
  //     currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
  //     conflicts.add(currentConflict);
  //   }

  //   // Conflicts were found so remove added data if all or nothing flag is set
  //   if (allOrNothing && !conflicts.isEmpty()) {
  //     remove(insertedData);
  //   }
  //   return conflicts;
  // }



  // /**
  //  * Function which attempts to insert a list of points into the tree.
  //  * If any point fails to be inserted, then no points will be added
  //  * If the list of returned conflict spaces is not empty, 
  //  * then a conflict occurred and no points were added to the tree
  //  * 
  //  * @param points The list of downtrack, crosstrack, time points to add
  //  * 
  //  * @return A list of detected conflicts between the provided points and the current tree contents
  //  */
  // public List<ConflictSpace> attemptToInsert(List<Point3D> points) {
  //   return insertWithConditions(points, noConflictCondition, true);
  // }

  // /**
  //  * Function which checks if a list of points can be inserted into the tree.
  //  * If the list of returned conflict spaces is not empty, 
  //  * then a conflict will occur and the attemptToInsert function will fail
  //  * 
  //  * @param points The list of downtrack, crosstrack, time points to check
  //  * 
  //  * @return A list of detected conflicts between the provided points and the current tree contents
  //  */
  // public List<ConflictSpace> checkInsert(List<Point3D> points) {
  //   return insertWithConditions(points, noInsertConditions, true);
  // }


  // // Returns a list containing all the datums that could be inserted
  // public List<HyperOcTreeDatum<T>> insert(List<Point3D> points) {
  //   List<HyperOcTreeDatum<T>> insertedData = new LinkedList<>();
  //   for (Point p: points) {
  //     // Try to insert the point
  //     HyperOcTreeDatum<T> datum = new HyperOcTreeDatum<T>(p, null);
  //     if (root.insert(datum, sizeConditions)) { // TODO maybe we should do the insert with the lane information as the datum
  //       insertedData.add(datum);
  //     }
  //   }
  //   return insertedData;
  // }

  // public void remove(List<HyperOcTreeDatum<T>> data) {
  //   if (root == null) {
  //     return;
  //   }
  //   for (HyperOcTreeDatum<T> datum: data) {
  //     // Try to remove the datum
  //     root.remove(datum);
  //   }
  // }
}