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
 * Basic data class for the {@link IntervalTree}
 * <p>
 * Stores a generic data field the start and end points of the corresponding interval.
 * The inclusive/exclusiveness of those start and end points is determined by the IntervalTree's
 * {@link IntervalCalculatorStrategy}
 */
public class Interval<V> {
    private V data;
    private double start;
    private double end;

    public Interval(V data, double start, double end) {
        this.data = data;
        this.start = start;
        this.end = end;
    }

    /**
     * Constructor to build intervals for use in queries, do not attempt to insert into trees
     */
    public Interval(double start, double end) {
        this.data = null;
        this.start = start;
        this.end = end;
    }

    /**
     * Get the data value associated with the interval
     */
    public V getData() {
        return data;
    }

    /**
     * Get the start point of this interval.
     * <p>
     * The inclusive/exclusiveness of this interval depends on the tree it is contained in and its
     * configured {@link IntervalCalculatorStrategy} instance
     */
    public double getStart() {
        return start;
    }

    /**
     * Get the end point of this interval.
     * <p>
     * The inclusive/exclusiveness of this interval depends on the tree it is contained in and its
     * configured {@link IntervalCalculatorStrategy} instance
     */
    public double getEnd() {
        return end;
    }

    @Override
    public String toString() {
        return "Interval{data=" + data + ", start=" + start + ", end=" + end + "}";
    }
}
