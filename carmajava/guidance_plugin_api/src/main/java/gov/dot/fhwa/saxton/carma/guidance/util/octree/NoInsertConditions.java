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

/**
 * Insert strategy for not allowing intervals in an interval tree to overlap
 */
public class NoInsertConditions<T> implements HyperOcTreeConditions<T> {

  MaxSizeConditions<T> maxSizeConditions;

  // Just make this package private
  protected NoInsertConditions(double minSizes[]) {
    maxSizeConditions = new MaxSizeConditions<>(minSizes);
  }

  /**
   * When using this condition the insert will end when it reaches the minimum region size
   */
	@Override
	public boolean doneInsert(HyperOcTreeNode<T> node, HyperOcTreeDatum<T> datum) {
    // If the node cannot be subdivided add the datum to this node
    return maxSizeConditions.doneInsert(node, datum);
  }

  /**
   * When using this condition it is illegal to insert a datum into a cell containing another datum
   */

  public boolean validInsert(HyperOcTreeNode<T> currentNode, HyperOcTreeDatum<T> datum) {
    return currentNode.contents.isEmpty();
  }

        /**
   * 
   */
  @Override
  public void performInsertion(HyperOcTreeNode<T> currentNode, HyperOcTreeDatum<T> datum) {
    // do nothing
  }
}