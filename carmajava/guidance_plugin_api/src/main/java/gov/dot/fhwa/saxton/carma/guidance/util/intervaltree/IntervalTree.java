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

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Data structure for the storage and usage of interval-based information.
 * <p>
 * Intervals can be any range of double values and may be closed or open at
 * their ends. Uses strategy pattern objects to determine interval open/closedness
 * and insert behavior of tree.
 * <p>
 * Construct using {@link IntervalTreeFactory}
 */
public class IntervalTree<T> {
    protected IntervalTreeNode<T> root;
    protected IntervalCalculatorStrategy<T> intCalculator;
    protected IntervalTreeInsertStrategy<T> insertStrategy;

    protected IntervalTree(IntervalCalculatorStrategy<T> intCalculator, IntervalTreeInsertStrategy<T> insertStrategy) {
        this.intCalculator = intCalculator;
        this.insertStrategy = insertStrategy;
    }

    /**
     * Attempt to insert the value in the tree according to the configured insertion strategy.
     * <p>
     * Only accepts Intervals with non-null data fields, use Optional<T> if data may be missing
     * 
     * @return True if the insertion was successful, false o.w.
     */
    public boolean insert(Interval<T> value) {
        if (value.getData() == null) {
            // Reject an interval with null data
            return false;
        }

        if (root == null) {
            root = new IntervalTreeNode<T>(intCalculator, insertStrategy, (value.getStart() + value.getEnd()) / 2.0);
        }

        return root.insert(value);
    }

    /**
     * Get a sorted set (ordered according to the configured IntervalCalculatorStrategy) of intervals that
     * intersect (according to the IntervalCalculatorStrategy) with the input interval in the tree
     * 
     * @return A sorted set containing any found intersections, empty if nothing found
     */
    public SortedSet<Interval<T>> findIntersectionsWith(Interval<T> interval) {
        if (root == null) {
            return new TreeSet<>();
        }

        return root.findIntersectionsWith(interval);
    }

    /**
     * Get a sorted set (ordered according to the configured IntervalCalculatorStrategy) of intervals that
     * intersect (according to the IntervalCalculatorStrategy) with the input point in the tree
     * 
     * @return A sorted set containing any found intersections, empty if nothing found
     */
    public SortedSet<Interval<T>> findIntersectionsWith(double pt) {
        if (root == null) {
            return new TreeSet<>();
        }

        return root.findIntersectionsWith(pt);
    }

    /**
     * Get the sorted set of intervals in this tree
     */
    public SortedSet<Interval<T>> getIntervals() {
        SortedSet<Interval<T>> out = new TreeSet<>(intCalculator::compareIntervals);
        if (root != null) {
            out.addAll(root.getIntervals());
        }

        return out;
    }


    /**
     * Flatten the tree into a sorted list of raw elements, sorted according to the configured
     * IntervalCalculatorStrategy. Discards all interval data in the output.
     */
    public List<T> toSortedList() {
        List<T> out = new ArrayList<>();
        if (root != null) {
            out.addAll(root.toSortedList());
        }

        return out;
    }
}