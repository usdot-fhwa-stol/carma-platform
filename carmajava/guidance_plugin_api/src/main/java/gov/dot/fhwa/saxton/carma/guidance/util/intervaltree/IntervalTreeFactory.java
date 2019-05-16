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
 * Factory/builder pattern class for the construction of IntervalTree instances.
 * <p>
 * Assigns the strategies needed by the IntervalTree instance.
 */
public class IntervalTreeFactory {
    /**
     * Build an interval tree with the default configuration.
     * <p>
     * Uses the {@link ClosedStartOpenEndIntervalCalculator} and {@link NonOverlappingInsertStrategy} to
     * create a tree that contains intervals closed on their start an open on their end points that also
     * fails if an insert is attempted that overlaps another interval already in the tree
     */
    public static <T> IntervalTree<T> buildIntervalTree() {
        return new IntervalTree<>(new ClosedStartOpenEndIntervalCalculator<T>() , new NonOverlappingInsertStrategy<T>());
    }
}