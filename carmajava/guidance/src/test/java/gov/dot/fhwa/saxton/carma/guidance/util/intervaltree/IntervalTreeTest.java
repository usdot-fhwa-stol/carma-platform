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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.SortedSet;


public class IntervalTreeTest {
    private IntervalTree<String> tree;

    @Before
    public void setup() {
        tree = IntervalTreeFactory.buildIntervalTree();
    }

    @Test
    public void testInsertRootNode() {
        assertNull(tree.root);

        Interval<String> datum = new Interval<String>("test", 0.0, 1.0);

        boolean result = tree.insert(datum);
        assertTrue(result);
        assertNotNull(tree.root);
        assertEquals("test", tree.root.data.first().getData());
    }

    @Test
    public void testInsertRootNodeWithOverlap() {
        tree.insert(new Interval<String>("test", 0.0, 1.0));
        boolean result = tree.insert(new Interval<String>("test", 0.0, 1.0));

        assertFalse(result);
        assertEquals("test", tree.root.data.first().getData());
        assertEquals(1, tree.root.data.size());
    }

    @Test
    public void testInsertMultipleNodes() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        assertTrue(result);
        result = tree.insert(new Interval<String>("test", 0.0, 1.0));
        assertTrue(result);

        result = tree.insert(new Interval<String>("test", 10.0, 11.0));
        assertTrue(result);

        System.out.println(tree);
    }

    @Test
    public void testInsertMultipleNodesWithOverlap() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        assertTrue(result);
        result = tree.insert(new Interval<String>("test", 0.0, 1.0));
        assertTrue(result);

        result = tree.insert(new Interval<String>("test", 10.0, 11.0));
        assertTrue(result);

        result = tree.insert(new Interval<String>("test", 3.0, 4.1));
        assertFalse(result);

        System.out.println(tree);
    }

    @Test
    public void testEmptyFindIntersections() {
        assertTrue(tree.findIntersectionsWith(new Interval<String>("test", 0.0, 10.0)).isEmpty());
    }

    @Test
    public void testFindIntersectionsWithSingleInterval() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        result = tree.insert(new Interval<String>("test1", 0.0, 1.0));

        result = tree.insert(new Interval<String>("test2", 10.0, 11.0));

        SortedSet<Interval<String>> intersects = tree.findIntersectionsWith(new Interval<String>("test", 3.0, 4.5));
        assertFalse(intersects.isEmpty());
        assertEquals(1, intersects.size());

        assertEquals("test", intersects.first().getData());
    }

    @Test
    public void testFindIntersectionsWithMultipleIntervals() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        result = tree.insert(new Interval<String>("test1", 0.0, 1.0));

        result = tree.insert(new Interval<String>("test2", 10.0, 11.0));

        SortedSet<Interval<String>> intersects = tree.findIntersectionsWith(new Interval<String>("test", 3.0, 10.5));
        assertFalse(intersects.isEmpty());
        assertEquals(2, intersects.size());

        boolean testFound = false;
        boolean test2Found = false;
        for (Interval<String> inter : intersects) {
            if (inter.getData().equals("test")) {
                testFound = true;
            } else if (inter.getData().equals("test2")) {
                test2Found = true;
            }
        }

        assertTrue(testFound);
        assertTrue(test2Found);
    }

    @Test
    public void testFindIntersectionsWithMultipleIntervalsFail() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        result = tree.insert(new Interval<String>("test1", 0.0, 1.0));

        result = tree.insert(new Interval<String>("test2", 10.0, 11.0));

        SortedSet<Interval<String>> intersects = tree.findIntersectionsWith(new Interval<String>("test", 3.0, 3.5));
        assertTrue(intersects.isEmpty());
    }

    @Test
    public void testEmptyFindIntersectionsWithPoint() {
        assertTrue(tree.findIntersectionsWith(4.5).isEmpty());
    }

    @Test
    public void testFindIntersectionsWithSingleIntervalWithPoint() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        result = tree.insert(new Interval<String>("test1", 0.0, 1.0));

        result = tree.insert(new Interval<String>("test2", 10.0, 11.0));

        SortedSet<Interval<String>> intersects = tree.findIntersectionsWith(4.5);
        assertFalse(intersects.isEmpty());
        assertEquals(1, intersects.size());

        assertEquals("test", intersects.first().getData());
    }

    @Test
    public void testFindIntersectionsWithSingleIntervalWithPointFail() {
        boolean result = tree.insert(new Interval<String>("test", 4.0, 5.0));
        result = tree.insert(new Interval<String>("test1", 0.0, 1.0));

        result = tree.insert(new Interval<String>("test2", 10.0, 11.0));

        SortedSet<Interval<String>> intersects = tree.findIntersectionsWith(5.5);
        assertTrue(intersects.isEmpty());
    }
}