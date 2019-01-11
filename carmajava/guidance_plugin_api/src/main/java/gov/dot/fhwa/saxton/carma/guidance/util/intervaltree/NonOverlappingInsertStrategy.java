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

package gov.dot.fhwa.saxton.carma.guidance.util.intervaltree;

/**
 * Insert strategy for not allowing intervals in an interval tree to overlap
 */
public class NonOverlappingInsertStrategy<T> implements IntervalTreeInsertStrategy<T> {

    // Just make this package private
    protected NonOverlappingInsertStrategy() {}

	@Override
	public boolean insert(IntervalTreeNode<T> node, Interval<T> datum) {
        if (node.data.isEmpty()) {
            // No data stored in this node currently
            node.data.add(datum);
            return true;
        }

        int cmp = node.intervalCalculator.compareIntervals(datum, node.data.first());
        if (cmp < 0) {
            // This interval exists wholly to the left of center

            // Create our left child if needed
            if (node.left == null) {
                node.left = new IntervalTreeNode<T>(node.intervalCalculator, node.insertStrategy, datum.getStart() + (datum.getEnd() - datum.getStart()) / 2.0);
            }

            return node.left.insert(datum);
        } else if (cmp == 0) {
            // This interval intersects with the interval stored in this node, we can't insert it
            return false;
        } else {
            // This interval exists wholly to the right of center

            // Create our right child if needed
            if (node.right == null) {
                node.right = new IntervalTreeNode<T>(node.intervalCalculator, node.insertStrategy, datum.getStart() + (datum.getEnd() - datum.getStart()) / 2.0);
            }

            return node.right.insert(datum);
        }
	}

}