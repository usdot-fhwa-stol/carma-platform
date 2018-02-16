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

package gov.dot.fhwa.saxton.carma.guidance.util.intervaltree;

import java.util.ArrayList;
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
public class IntervalTreeNode<T> {
    protected double center;
    protected IntervalTreeNode<T> left;
    protected IntervalTreeNode<T> right;
    protected SortedSet<Interval<T>> data;
    protected IntervalCalculatorStrategy<T> intervalCalculator;
    protected IntervalTreeInsertStrategy<T> insertStrategy;

    protected IntervalTreeNode(IntervalCalculatorStrategy<T> intervalCalculator,
            IntervalTreeInsertStrategy<T> insertStrategy, double center) {
        this.intervalCalculator = intervalCalculator;
        this.insertStrategy = insertStrategy;
        this.center = center;
        data = new TreeSet<>(intervalCalculator::compareIntervals);
    }

    protected IntervalTreeNode(IntervalCalculatorStrategy<T> intervalCalculator,
            IntervalTreeInsertStrategy<T> insertStrategy, double center, IntervalTreeNode<T> left,
            IntervalTreeNode<T> right) {
        this(intervalCalculator, insertStrategy, center);
        this.left = left;
        this.right = right;
    }

    protected IntervalTreeNode(IntervalCalculatorStrategy<T> intervalCalculator,
            IntervalTreeInsertStrategy<T> insertStrategy, double center, IntervalTreeNode<T> left,
            IntervalTreeNode<T> right, SortedSet<Interval<T>> data) {
        this(intervalCalculator, insertStrategy, center, left, right);
        this.data = data;
    }

    /**
     * Attempt to insert the value under this node in the tree according to the configured insertion strategy.
     * 
     * @return True if the insertion was successful, false o.w.
     */
    public boolean insert(Interval<T> value) {
        return insertStrategy.insert(this, value);
    }

    /**
     * Get a sorted set (ordered according to the configured IntervalCalculatorStrategy) of intervals that
     * intersect (according to the IntervalCalculatorStrategy) with the input interval under this node in the
     * tree.
     * 
     * @return A sorted set containing any found intersections, empty if nothing found
     */
    public SortedSet<Interval<T>> findIntersectionsWith(Interval<T> interval) {
        SortedSet<Interval<T>> result = new TreeSet<>(intervalCalculator::compareIntervals);

        int cmp = (data.size() > 1 ? intervalCalculator.compareIntervalAndPoint(interval, center)
                : intervalCalculator.compareIntervals(interval, data.first()));
        if (cmp < 0) {
            // Interval exists wholly to the left, recurse
            if (left != null) {
                result.addAll(left.findIntersectionsWith(interval));
            }
        }
        if (cmp == 0) {
            // Check for any intersections in our node's data
            double leftSpan = Double.POSITIVE_INFINITY;
            double rightSpan = Double.NEGATIVE_INFINITY;
            for (Interval<T> inter : data) {
                leftSpan = Math.min(leftSpan, inter.getStart());
                rightSpan = Math.max(rightSpan, inter.getEnd());

                if (intervalCalculator.checkIntersection(interval, inter)) {
                    result.add(inter);
                }
            }

            // Then, if the intersection interval extends beyond our node's data to the left or right, recurse
            if (interval.getStart() <= leftSpan && left != null) {
                result.addAll(left.findIntersectionsWith(interval));
            }
            if (interval.getEnd() >= rightSpan && right != null) {
                result.addAll(right.findIntersectionsWith(interval));
            }
        } else {
            // Interval exists wholly to the right, recurse
            if (right != null) {
                result.addAll(right.findIntersectionsWith(interval));
            }
        }

        return result;
    }

    /**
     * Get a sorted set (ordered according to the configured IntervalCalculatorStrategy) of intervals that
     * intersect (according to the IntervalCalculatorStrategy) with the input point under this node in the
     * tree.
     * 
     * @return A sorted set containing any found intersections, empty if nothing found
     */
    public SortedSet<Interval<T>> findIntersectionsWith(double point) {
        SortedSet<Interval<T>> result = new TreeSet<>(intervalCalculator::compareIntervals);
        // Check for any intersections in our node's data
        double leftSpan = Double.POSITIVE_INFINITY;
        double rightSpan = Double.NEGATIVE_INFINITY;
        for (Interval<T> inter : data) {
            leftSpan = Math.min(leftSpan, inter.getStart());
            rightSpan = Math.max(rightSpan, inter.getEnd());

            if (intervalCalculator.checkIntersection(inter, point)) {
                result.add(inter);
            }
        }

        // Then, if the point exists beyond our node's data to the left or right, recurse
        if (point <= leftSpan && left != null) {
            result.addAll(left.findIntersectionsWith(point));
        }
        if (point >= rightSpan && right != null) {
            result.addAll(right.findIntersectionsWith(point));
        }

        return result;
    }

    /**
     * Get a sorted set of all intervals sorted below this in the tree by doing a pre-order traversal
     */
    public SortedSet<Interval<T>> getIntervals() {
        SortedSet<Interval<T>> out = new TreeSet<>(intervalCalculator::compareIntervals);
        if (left != null) {
            out.addAll(left.getIntervals());
        }

        out.addAll(data);

        if (right != null) {
            out.addAll(right.getIntervals());
        }

        return out;
    }

    /**
     * Flatten the tree below this node into a sorted list of raw elements by performing a
     * pre-order traversal
     */
    public List<T> toSortedList() {
        List<T> out = new ArrayList<>();
        if (left != null) {
            out.addAll(left.toSortedList());
        }

        for (Interval<T> interval : data) {
            out.add(interval.getData());
        }

        if (right != null) {
            out.addAll(right.toSortedList());
        }

        return out;
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