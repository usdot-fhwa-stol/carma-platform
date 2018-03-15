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

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 */
public class ConflictDetectorOcTree<T> {
  protected HyperOcTreeNode<T> root;
  protected HyperOcTreeConditions<T> conditions;

  protected ConflictDetectorOcTree(HyperOcTreeConditions<T> conditions) {
    this.conditions = conditions;
  }

  /**
   * Attempt to insert the value in the tree according to the configured insertion strategy.
   * <p>
   * Only accepts Intervals with non-null data fields, use Optional<T> if data may be missing
   * 
   * @return True if the insertion was successful, false o.w.
   */
  public boolean insert(HyperOcTreeDatum<T> value) {
    // TODO
    // Think about how what the return from an insert should be (A list of conflicts?)
    // If it is not true / false perhaps this should be broken into a strategy or a differently named class
    return false;
  }
}