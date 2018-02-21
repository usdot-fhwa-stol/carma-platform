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

/**
 * Interface for providing methods to check whether points or intervals
 * intersect in an {@link IntervalTree}
 */
public interface IntervalCalculatorStrategy<T> {
    /**
     * Check if the specified intervals intersect.
     * <p>
     * Intervals are defined as intersecting if they share at least one point between them,
     * that is inter1 n inter2 != {}.
     */
    boolean checkIntersection(Interval<T> inter1, Interval<T> inter2);

    /**
     * Check if the specified interval and the specified point intersect.
     * <p>
     * Intervals and a point are defined as intersecting if point is an element of the interval,
     * that is inter n {pt} = {pt}.
     */
    boolean checkIntersection(Interval<T> inter, double pt);

    /**
     * Compare the specified intervals to determine which is first, ordinally.
     * 
     * @returns -1 if inter1 is wholly before inter2, 0 if they are the intersect, +1 if inter1 is wholly after inter2
     */
    int compareIntervals(Interval<T> inter1, Interval<T> inter2);

    /**
     * Compare the specified interval and the specified point to determine their order
     * 
     * @returns -1 if inter is before the point, 0 if the point is inside inter, +1 if inter is afer the point
     */
    int compareIntervalAndPoint(Interval<T> inter1, double pt);
}