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
public class MaxSizeInsertStrategy<T> implements OcTreeInsertStrategy<T> {

  // Just make this package private
  protected MaxSizeInsertStrategy() {}
  protected double minSizes[] = {4,4,0.1};

	@Override
	public boolean insert(OcTreeNode<T> node, OcTreeDatum<T> datum) {
    if (!node.contains(datum.getPoint())) {
      return false;
    }
    // If the node cannot be subdivided add the datum to this node
    if (node.region.canFitInside(minSizes, 0.5)) {
      node.contents.add(datum);
      return true;
    }
    // If this node has no children create them
    if (node.children.isEmpty()) { 
      // No data stored in this node currently
      node.divide();
      // No need to move this node's contents to children as children will only be stored in minimum sized nodes
      // A node without children will therefore not have any contents
    }

    // Continue to insert. Only one child will actually have the datum inserted
    for (OcTreeNode child: node.children) {
      if (insert(child, datum)) {
        return true;
      }
    }

      return false;
  }
}