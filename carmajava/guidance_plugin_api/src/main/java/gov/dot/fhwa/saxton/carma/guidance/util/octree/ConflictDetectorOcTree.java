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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import cav_msgs.RouteSegment;

/**
 */
public class ConflictDetectorOcTree<T> {
  protected HyperOcTreeNode<T> root;
  protected HyperOcTreeConditions<T> conditions;
  private static final int DOWNTRACK_IDX = 0;
  private static final int CROSSTRACK_IDX = 1;
  private static final int TIME_IDX = 2;
  private double[] maxSize = {4,4,0.1};
  
  HyperOcTreeConditions<T> noConflictCondition = new NoConflictConditions<>(maxSize);
  HyperOcTreeConditions<T> sizeConditions = new MaxSizeConditions<>(maxSize);


  protected ConflictDetectorOcTree(HyperOcTreeConditions<T> conditions) {
    this.conditions = conditions;
  }

  public List<ConflictSpace> attemptToInsert(List<Point> points) {

    List<Point> insertedPoints = new LinkedList<>();
    List<ConflictSpace> conflicts = new LinkedList<>();
    ConflictSpace currentConflict = null;
    int lane = 0; // TODO lane for each point
    Point prevPoint = null;
    for (Point p: points) {
      // Failure to insert the point will mean a conflict has occurred
      if (!root.insert(new HyperOcTreeDatum<T>(p, null), noConflictCondition)) { // TODO maybe we should do the insert with the lane information as the datum
        // If no conflict is being tracked this is a new conflict
        if (currentConflict == null) {
          currentConflict = new ConflictSpace(p.getDim(DOWNTRACK_IDX), p.getDim(TIME_IDX), lane);
        } else if (lane != currentConflict.getLane()) {
          // If we are tracking a conflict but the lane has changed then end that conflict and create a new one
          currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
          currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
          conflicts.add(currentConflict);
          // Use the current point's lane but the previous points distance and time to define the start of the new conflict
          currentConflict = new ConflictSpace(prevPoint.getDim(DOWNTRACK_IDX), prevPoint.getDim(TIME_IDX), lane);
        } 
      } else {
        insertedPoints.add(p); // Add the inserted point to the list of inserted points
        // If we could insert the point but we are tracking a conflict then that conflict is done
        if (currentConflict != null) {
          currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
          currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
          conflicts.add(currentConflict);
          currentConflict = null; // Stop tracking the conflict
        }
      }
      prevPoint = p;
    }
    // Close the currentConflict if it was extending past the last point
    if (currentConflict != null) {
      currentConflict.setEndDowntrack(prevPoint.getDim(DOWNTRACK_IDX));
      currentConflict.setEndTime(prevPoint.getDim(TIME_IDX));
      conflicts.add(currentConflict);
    }
    //TODO delete the inserted points if there was a conflict (OR maybe don't actually insert them? that would take some thought)
    return conflicts;
  }


  public boolean insert(List<Point> points) {
    for (Point p: points) {
      // Try to insert the point
      if (!root.insert(new HyperOcTreeDatum<T>(p, null), sizeConditions)) { // TODO maybe we should do the insert with the lane information as the datum
        return false;
      }
    }
    return true;
  }
  // /**
  //  * Attempt to insert the value in the tree according to the configured insertion strategy.
  //  * <p>
  //  * Only accepts Intervals with non-null data fields, use Optional<T> if data may be missing
  //  * 
  //  * @return True if the insertion was successful, false o.w.
  //  */
  // public boolean insert(HyperOcTreeDatum<T> value) {
  //   // TODO
  //   // Think about how what the return from an insert should be (A list of conflicts?)
  //   // If it is not true / false perhaps this should be broken into a strategy or a differently named class
  //   return false;
  // }
}