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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.*;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.NSpatialHashMap;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.NSpatialHashStrategy;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.spatialstructure.SimpleHashStrategy;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Unit tests for the CartesianObject class
 */
public class NSpatialHashMapTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(NSpatialHashMapTest.class);
    log.info("Setting up tests for NSpatialHashMap");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testInsert() throws Exception {
    // Test 2D
    double[] cellSizes = {2,2};
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();

    // Test 2D Object
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(4,4),
      new Point2D(7,4),
      new Point2D(4,2),
      new Point2D(7,2)
    ));
    CartesianObject obj = new CartesianObject(points);
    CartesianObject obj2 = new CartesianObject(points);

    map.insert(obj);
    NSpatialHashKey key1 = new NSpatialHashKey(new long[]{2,2});
    NSpatialHashKey key2 = new NSpatialHashKey(new long[]{3,2});
    NSpatialHashKey key3 = new NSpatialHashKey(new long[]{2,1});
    NSpatialHashKey key4 = new NSpatialHashKey(new long[]{3,1});
    assertTrue(map.getKeys().contains(key1));
    assertTrue(map.getKeys().contains(key2));
    assertTrue(map.getKeys().contains(key3));
    assertTrue(map.getKeys().contains(key4));

    // Test multiple objects
    map.insert(obj2);
    assertEquals(4, map.getObjects().size());
    ArrayList<List<CartesianObject>> objectLists = new ArrayList<>(map.getObjects());
    assertTrue(objectLists.get(0).contains(obj));
    assertTrue(objectLists.get(1).contains(obj));
    assertTrue(objectLists.get(2).contains(obj));
    assertTrue(objectLists.get(3).contains(obj));

    assertTrue(objectLists.get(0).contains(obj2));
    assertTrue(objectLists.get(1).contains(obj2));
    assertTrue(objectLists.get(2).contains(obj2));
    assertTrue(objectLists.get(3).contains(obj2));

    // Test overlapping object
    List<? extends Point> points2 = new LinkedList<>(Arrays.asList(
      new Point2D(6.5,4.5),
      new Point2D(7.5,3.5),
      new Point2D(6.5,3.5),
      new Point2D(7.5,4.5)
    ));

    CartesianObject obj3 = new CartesianObject(points2);
    map.insert(obj3);
    // The bins should be unchanged
    assertEquals(4, map.getKeys().size());
    objectLists = new ArrayList<>(map.getObjects());
    int cellsWithObject = 0;
    for (List<CartesianObject> objects: objectLists) {
      if( objects.contains(obj3))
        cellsWithObject++;
    }
    assertEquals(2, cellsWithObject);

    // Test single point object inserts to the same key as that point
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map
    points = new LinkedList<>(Arrays.asList(
      new Point2D(4,4)
    ));
    obj = new CartesianObject(points);

    map.insert(obj);
    key1 = new NSpatialHashKey(new long[]{2,2});
    assertTrue(map.getKeys().contains(key1));
    assertTrue(key1.equals(new SimpleHashStrategy(cellSizes).getKey(new Point2D(4,4))));

    // Test 3D object that fits within a cell
    cellSizes = new double[]{2,2,2};
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map

    points = new LinkedList<>(Arrays.asList(
      new Point3D(4.5,4.5,4.5),
      new Point3D(4.6,4.6,4.6)
    ));

    obj = new CartesianObject(points);
    map.insert(obj);

    key1 = new NSpatialHashKey(new long[]{2,2,2});
    assertTrue(map.getKeys().contains(key1));

    // Test wrong dimension
    List<? extends Point> wrongPoints = new LinkedList<>(Arrays.asList(
      new Point(4),
      new Point(4),
      new Point(2),
      new Point(2)
    ));

    CartesianObject badObj = new CartesianObject(wrongPoints);
    assertFalse(map.insert(badObj));
  }

  /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testGetCollisionsWithPoint() throws Exception {
    // Test 2D
    double[] cellSizes = {1,1};
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();


    // Test No Intersect 2D Objects and Point
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(4,0),
      new Point2D(4,3),
      new Point2D(1,3)
    ));
    CartesianObject obj = new CartesianObject(points);
    CartesianObject obj2 = new CartesianObject(points);

    map.insert(obj);
    map.insert(obj2);

    Point otherPoint = new Point2D(1,2);
    assertTrue(map.getCollisions(otherPoint).isEmpty());

    // Test Intersect 2D Objects and Point
    otherPoint = new Point2D(1.1,1);
    List<CartesianObject> collidingObjects = map.getCollisions(otherPoint);
    assertEquals(2, collidingObjects.size());
    assertEquals(collidingObjects.get(0), obj);
    assertEquals(collidingObjects.get(1), obj2);

    // Test No Intersect 3D
    cellSizes = new double[]{1,1,1};
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map

    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0, 7),
      new Point3D(4,0, 8),
      new Point3D(4,3, 9),
      new Point3D(1,3, 10)
    ));
    obj = new CartesianObject(points);

    map.insert(obj);
    otherPoint = new Point3D(-1,-1,0);
    assertTrue(map.getCollisions(otherPoint).isEmpty());

    // Test Intersect 3D
    otherPoint = new Point3D(2,1,7.1);
    assertEquals(map.getCollisions(otherPoint).get(0), obj);

    // Test No Intersect nd
    cellSizes = new double[]{1,1,1,1,1};
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map

    points = new LinkedList<>(Arrays.asList(
      new Point(1, 0, 7, 5, 3),
      new Point(4, 0, 6, 7, 5),
      new Point(4, 3, 8, 6, 4)
    ));

    obj = new CartesianObject(points);

    map.insert(obj);
    otherPoint = new Point(-2,-2,-2,-2,-2);
    assertTrue(map.getCollisions(otherPoint).isEmpty());

    // Test Intersect nd
    otherPoint = new Point(2,1,7.1, 5.5, 3.1);
    assertEquals(map.getCollisions(otherPoint).get(0), obj);

    // Test wrong dimension
    assertNull(map.getCollisions(new Point(4)));
  }

  /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testGetCollisionsWithObject() throws Exception {
    // Test 2D
    double[] cellSizes = {1,1};
    IIntersectionChecker aabbChecker = new AxisAlignedBoundingBox();
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();


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

    CartesianObject obj3 = new CartesianObject(points);

    map.insert(obj);
    map.insert(obj3);
    assertTrue(map.getCollisions(obj2).isEmpty());

    // Test Intersect 2D
    obj2 = new CartesianObject(points);

    List<CartesianObject> collidingObjects = map.getCollisions(obj2);
    assertEquals(2, collidingObjects.size());
    assertTrue(collidingObjects.contains(obj));
    assertTrue(collidingObjects.contains(obj3));

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

    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map
    map.insert(obj);

    collidingObjects = map.getCollisions(obj2);
    assertEquals(1, collidingObjects.size());
    assertEquals(collidingObjects.get(0), obj);

    // Test intersection of 2 lines at tip
    points2 = new LinkedList<>(Arrays.asList(
      new Point2D(1,1),
      new Point2D(1,2),
      new Point2D(1,3)
    ));
    obj2 = new CartesianObject(points2);

    collidingObjects = map.getCollisions(obj2);
    assertEquals(1, collidingObjects.size());
    assertEquals(collidingObjects.get(0), obj);

    // Test not intersection of 2 lines
    points2 = new LinkedList<>(Arrays.asList(
      new Point2D(1,2),
      new Point2D(1,3),
      new Point2D(1,4)
    ));
    obj2 = new CartesianObject(points2);

    collidingObjects = map.getCollisions(obj2);
    assertTrue(collidingObjects.isEmpty());

    // Test No Intersect 3D
    cellSizes = new double[]{1,1,1};
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map

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

    map.insert(obj);
    assertTrue(map.getCollisions(obj2).isEmpty());

    // Test Intersect 3D
    obj2 = new CartesianObject(points);

    collidingObjects = map.getCollisions(obj2);

    assertEquals(1, collidingObjects.size());
    assertEquals(collidingObjects.get(0), obj);

    // Test No Intersect nd
    cellSizes = new double[]{1,1,1,1,1};
    factory = new NSpatialHashMapFactory(cellSizes);
    map = factory.buildSpatialStructure(); // Reset map

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
    
    map.insert(obj);
    assertTrue(map.getCollisions(obj2).isEmpty());

    // Test Intersect nd
    obj2 = new CartesianObject(points);

    collidingObjects = map.getCollisions(obj2);

    assertEquals(1, collidingObjects.size());
    assertEquals(collidingObjects.get(0), obj);

    // Test wrong dimension
    List<? extends Point> wrongPoints = new LinkedList<>(Arrays.asList(
      new Point(4),
      new Point(4),
      new Point(2),
      new Point(2)
    ));

    CartesianObject badObj = new CartesianObject(wrongPoints);
    assertNull(map.getCollisions(badObj));
  }

  /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testRemove() throws Exception {
    // Test 2D
    double[] cellSizes = {2,2};
    IIntersectionChecker aabbChecker = new AxisAlignedBoundingBox();
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();
    
    // Test Removing object
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(4,4),
      new Point2D(7,4),
      new Point2D(4,2),
      new Point2D(7,2)
    ));
    CartesianObject obj = new CartesianObject(points);

    map.insert(obj);
    ArrayList<List<CartesianObject>> objectLists = new ArrayList<>(map.getObjects());
    assertTrue(objectLists.get(0).contains(obj));
    assertTrue(objectLists.get(1).contains(obj));
    assertTrue(objectLists.get(2).contains(obj));
    assertTrue(objectLists.get(3).contains(obj));

    map.remove(obj);
    objectLists = new ArrayList<>(map.getObjects());
    assertFalse(objectLists.get(0).contains(obj));
    assertFalse(objectLists.get(1).contains(obj));
    assertFalse(objectLists.get(2).contains(obj));
    assertFalse(objectLists.get(3).contains(obj));

    // Test wrong dimension
    List<? extends Point> wrongPoints = new LinkedList<>(Arrays.asList(
      new Point(4),
      new Point(4),
      new Point(2),
      new Point(2)
    ));

    CartesianObject obj2 = new CartesianObject(wrongPoints);
    assertFalse(map.remove(obj2));
  }

    /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testSurrounds() throws Exception {
    // Test 2D
    double[] cellSizes = {2,2};
    IIntersectionChecker aabbChecker = new AxisAlignedBoundingBox();
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();

    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(0,2),
      new Point2D(2,0),
      new Point2D(0,0),
      new Point2D(2,2)
    ));
    CartesianObject obj = new CartesianObject(points);

    map.insert(obj);

    // Test a surrounded point
    assertTrue(map.surrounds(new Point2D(1,1)));

    // Test a non enclosed point
    assertFalse(map.surrounds(new Point2D(-1,-1)));

    // Test a corner point. Should be exclusive
    assertFalse(map.surrounds(new Point2D(0,0)));

    // Test wrong dimension
    assertFalse(map.surrounds(new Point(0,0,0)));
  }

      /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testGetBounds() throws Exception {
    // Test 2D
    double[] cellSizes = {2,2};
    IIntersectionChecker aabbChecker = new AxisAlignedBoundingBox();
    NSpatialHashMapFactory factory = new NSpatialHashMapFactory(cellSizes);
    NSpatialHashMap map = factory.buildSpatialStructure();

    assertNull(map.getBounds());

    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(0,3),
      new Point2D(2,-1),
      new Point2D(0,-1),
      new Point2D(2,3)
    ));
    CartesianObject obj = new CartesianObject(points);

    map.insert(obj);

    double[][] bounds = map.getBounds();
    assertEquals(0, bounds[0][map.MIN_BOUND_IDX], 0.000000001);
    assertEquals(2, bounds[0][map.MAX_BOUND_IDX], 0.000000001);
    assertEquals(-1, bounds[1][map.MIN_BOUND_IDX], 0.000000001);
    assertEquals(3, bounds[1][map.MAX_BOUND_IDX], 0.000000001);
    
    List<? extends Point> points2 = new LinkedList<>(Arrays.asList(
      new Point2D(-3, 5),
      new Point2D(7, 8)
    ));
    CartesianObject obj2 = new CartesianObject(points2);

    map.insert(obj2);
    bounds = map.getBounds();
    assertEquals(-3, bounds[0][map.MIN_BOUND_IDX], 0.000000001);
    assertEquals(7, bounds[0][map.MAX_BOUND_IDX], 0.000000001);
    assertEquals(-1, bounds[1][map.MIN_BOUND_IDX], 0.000000001);
    assertEquals(8, bounds[1][map.MAX_BOUND_IDX], 0.000000001);
  }
}
