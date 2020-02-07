/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.*;
import org.junit.Test;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Unit tests for the AxisAlignedBoundingBox intersection checking strategy class
 */
public class AxisAlignedBoundingBoxTest {

  /**
   * Tests the factory and constructors
   * @throws Exception
   */
  @Test
  public void testFactory() throws Exception {

    IntersectionCheckerFactory factory = new IntersectionCheckerFactory();
    // Get an AxisAlignedBoundingBox intersection checker
    IIntersectionChecker checker = factory.buildChecker("AABB");
    assertEquals(AxisAlignedBoundingBox.class, checker.getClass());

    // Default collision checker is the AxisAlignedBoundingBox
    checker = factory.buildChecker("Fake class");
    assertEquals(AxisAlignedBoundingBox.class, checker.getClass());
  }

  /**
   * Tests the intersection detection of an object and a point
   * @throws Exception
   */
  @Test
  public void testIntersectionObj2Point() throws Exception {
    IntersectionCheckerFactory factory = new IntersectionCheckerFactory();
    // Get an AxisAlignedBoundingBox intersection checker
    IIntersectionChecker checker = factory.buildChecker("AABB");
    assertEquals(AxisAlignedBoundingBox.class, checker.getClass());

    // Test No Intersect 2D
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(4,0),
      new Point2D(4,3),
      new Point2D(1,3)
    ));
    CartesianObject obj = new CartesianObject(points);

    Point otherPoint = new Point2D(1,2);
    assertFalse(checker.intersects(obj, otherPoint));

    // Test Intersect 2D
    points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(4,0),
      new Point2D(4,3),
      new Point2D(1,3)
    ));
    obj = new CartesianObject(points);

    otherPoint = new Point2D(1.1,1);
    assertTrue(checker.intersects(obj, otherPoint));

    // Test No Intersect 3D
    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0, 7),
      new Point3D(4,0, 8),
      new Point3D(4,3, 9),
      new Point3D(1,3, 10)
    ));
    obj = new CartesianObject(points);
    otherPoint = new Point3D(-1,-1,0);
    assertFalse(checker.intersects(obj, otherPoint));

    // Test Intersect 3D
    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0, 7),
      new Point3D(4,0, 8),
      new Point3D(4,3, 9),
      new Point3D(1,3, 10)
    ));
    obj = new CartesianObject(points);
    otherPoint = new Point3D(2,1,7.1);
    assertTrue(checker.intersects(obj, otherPoint));

    // Test No Intersect nd
    points = new LinkedList<>(Arrays.asList(
      new Point(1, 0, 7, 5, 3),
      new Point(4, 0, 6, 7, 5),
      new Point(4, 3, 8, 6, 4)
    ));

    obj = new CartesianObject(points);
    otherPoint = new Point(-2,-2,-2,-2,-2);
    assertFalse(checker.intersects(obj, otherPoint));

    // Test Intersect nd
    points = new LinkedList<>(Arrays.asList(
      new Point(1, 0, 7, 5, 3),
      new Point(4, 0, 6, 7, 5),
      new Point(4, 3, 8, 6, 4)
    ));

    obj = new CartesianObject(points);
    otherPoint = new Point(2,1,7.1, 5.5, 3.1);
    assertTrue(checker.intersects(obj, otherPoint));

    // Test mismatched dimensions
    otherPoint = new Point2D(2,1);
    try {
      checker.intersects(obj, otherPoint);
      fail("Failed to catch exception for mismatched dimensions");
    } catch (IllegalArgumentException e) {
      assertTrue(1==1); // Expected exception to be thrown
    }
  }

  /**
   * Tests the intersection detection of an object and an object
   * @throws Exception
   */
  @Test
  public void testIntersectionObj2Obj() throws Exception {
    IntersectionCheckerFactory factory = new IntersectionCheckerFactory();
    // Get an AxisAlignedBoundingBox intersection checker
    IIntersectionChecker checker = factory.buildChecker("AABB");
    assertEquals(AxisAlignedBoundingBox.class, checker.getClass());

    // Test No Intersect 2D
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(4,0),
      new Point2D(4,3),
      new Point2D(1,3)
    ));
    CartesianObject obj = new CartesianObject(points);

    List<? extends Point> points2 = new LinkedList<>(Arrays.asList(
      new Point2D(-1,-1),
      new Point2D(-4,-1),
      new Point2D(-4,-3),
      new Point2D(-1,-3)
    ));
    CartesianObject obj2 = new CartesianObject(points2);

    assertFalse(checker.intersects(obj, obj2));

    // Test Intersect 2D
    obj2 = new CartesianObject(points);

    assertTrue(checker.intersects(obj, obj2));

    // Test edge intersection of square and line
    points = new LinkedList<>(Arrays.asList(
      // Line of 3 points
      new Point2D(1,1),
      new Point2D(2,1),
      new Point2D(3,1)
    ));
    obj = new CartesianObject(points);

    points2 = new LinkedList<>(Arrays.asList(
      new Point2D(1,1),
      new Point2D(2,1),
      new Point2D(2,2),
      new Point2D(1,2)
    ));
    obj2 = new CartesianObject(points2);

    assertTrue(checker.intersects(obj, obj2));

    // Test intersection of 2 lines at tip
    points2 = new LinkedList<>(Arrays.asList(
      new Point2D(1,1),
      new Point2D(1,2),
      new Point2D(1,3)
    ));
    obj2 = new CartesianObject(points2);

    assertTrue(checker.intersects(obj, obj2));

    // Test not intersection of 2 lines
    points2 = new LinkedList<>(Arrays.asList(
      new Point2D(1,2),
      new Point2D(1,3),
      new Point2D(1,4)
    ));
    obj2 = new CartesianObject(points2);

    assertFalse(checker.intersects(obj, obj2));

    // Test No Intersect 3D
    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0, 7),
      new Point3D(4,0, 8),
      new Point3D(4,3, 9),
      new Point3D(1,3, 10)
    ));
    obj = new CartesianObject(points);

    points2 = new LinkedList<>(Arrays.asList(
      new Point3D(-1,0, -7),
      new Point3D(-4,0, -8),
      new Point3D(-4,-3, -9),
      new Point3D(-1,-3, -10)
    ));
    obj2 = new CartesianObject(points2);

    assertFalse(checker.intersects(obj, obj2));

    // Test Intersect 3D
    obj2 = new CartesianObject(points);
    assertTrue(checker.intersects(obj, obj2));

    // Test No Intersect nd
    points = new LinkedList<>(Arrays.asList(
      new Point(1, 0, 7, 5, 3),
      new Point(4, 0, 6, 7, 5),
      new Point(4, 3, 8, 6, 4)
    ));

    obj = new CartesianObject(points);

    points2 = new LinkedList<>(Arrays.asList(
      new Point(-1, 0, -7, -5, -3),
      new Point(-4, 0, -6, -7, -5),
      new Point(-4, -3, -8, -6, -4)
    ));

    obj2 = new CartesianObject(points2);
    assertFalse(checker.intersects(obj, obj2));

    // Test Intersect nd
    obj2 = new CartesianObject(points);
    assertTrue(checker.intersects(obj, obj2));

    // Test mismatched dimensions
    points2 =  new LinkedList<>(Arrays.asList(new Point2D(2,1)));
    obj2 = new CartesianObject(points2);
    try {
      checker.intersects(obj, obj2);
      fail("Failed to catch exception for mismatched dimensions");
    } catch (IllegalArgumentException e) {
      assertTrue(1==1); // Expected exception to be thrown
    }
  }
}
