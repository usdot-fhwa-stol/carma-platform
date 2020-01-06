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
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Unit tests for the CartesianObject class
 */
public class CartesianObjectTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(CartesianObjectTest.class);
    log.info("Setting up tests for Vector");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the constructors
   * @throws Exception
   */
  @Test
  public void testConstructors() throws Exception {

    // Test 2D constructor
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(1,1),
      new Point2D(2,1)
    ));
    CartesianObject obj = new CartesianObject(points);
    assertEquals(2, obj.getNumDimensions());

    // Test 3D constructor
    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0,1),
      new Point3D(1,1,2),
      new Point3D(2,1,1)
    ));
    obj = new CartesianObject(points);
    assertEquals(3, obj.getNumDimensions());

    // Test nD constructor
    points = new LinkedList<>(Arrays.asList(
      new Point(1,0,1,4,5),
      new Point(1,0,1,3,7),
      new Point(1,0,1,4,4),
      new Point(1,0,1,3,3)
    ));
    obj = new CartesianObject(points);
    assertEquals(5, obj.getNumDimensions());

    // Test invalid input
    points = new LinkedList<>(Arrays.asList(
      new Point(1,0,1,4,5),
      new Point(1,0,1,3,7),
      new Point(1,0,1,4,4),
      new Point2D(1,0)
    ));
    try {
      obj = new CartesianObject(points);
      fail("Failed to catch exception for mismatched dimensions");
    } catch (IllegalArgumentException e) {
      assertTrue(1==1); // Expected exception to be thrown
    }
  }

  /**
   * Tests the bounds and centroid calculation
   * @throws Exception
   */
  @Test
  public void testBoundsAndCentroids() throws Exception {
    // Test 2D
    List<? extends Point> points = new LinkedList<>(Arrays.asList(
      new Point2D(1,0),
      new Point2D(4,0),
      new Point2D(4,3),
      new Point2D(1,3)
    ));
    CartesianObject obj = new CartesianObject(points);
    double[][] bounds = obj.getBounds();
    assertEquals(1.0, bounds[0][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(4.0, bounds[0][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(0.0, bounds[1][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(3.0, bounds[1][obj.getMaxBoundIndx()], 0.000000001);
    assertTrue(new Point(2.5, 6.0/4.0).almostEquals(obj.getCentroidOfCloud(), 0.00000001));
    assertTrue(new Point(2.5, 1.5).almostEquals(obj.getCentroidOfBounds(), 0.00000001));

    // Test 3D
    points = new LinkedList<>(Arrays.asList(
      new Point3D(1,0, 7),
      new Point3D(4,0, 8),
      new Point3D(4,3, 9),
      new Point3D(1,3, 10)
    ));
    obj = new CartesianObject(points);
    bounds = obj.getBounds();
    assertEquals(1.0, bounds[0][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(4.0, bounds[0][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(0.0, bounds[1][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(3.0, bounds[1][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(7.0, bounds[2][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(10.0, bounds[2][obj.getMaxBoundIndx()], 0.000000001);
    assertTrue(new Point(2.5, 6.0/4.0, 34.0/4.0).almostEquals(obj.getCentroidOfCloud(), 0.00000001));
    assertTrue(new Point(2.5, 1.5, 17.0/2.0).almostEquals(obj.getCentroidOfBounds(), 0.00000001));

    // Test nd
    points = new LinkedList<>(Arrays.asList(
      new Point(1, 0, 7, 5, 3),
      new Point(4, 0, 6, 7, 5),
      new Point(4, 3, 8, 6, 4)
    ));

    obj = new CartesianObject(points);
    bounds = obj.getBounds();
    assertEquals(1.0, bounds[0][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(4.0, bounds[0][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(0.0, bounds[1][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(3.0, bounds[1][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(6.0, bounds[2][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(8.0, bounds[2][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(5.0, bounds[3][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(7.0, bounds[3][obj.getMaxBoundIndx()], 0.000000001);
    assertEquals(3.0, bounds[4][obj.getMinBoundIndx()], 0.000000001);
    assertEquals(5.0, bounds[4][obj.getMaxBoundIndx()], 0.000000001);
    assertTrue(new Point(3.0, 1.0, 7.0, 6.0, 4.0).almostEquals(obj.getCentroidOfCloud(), 0.00000001));
    assertTrue(new Point(2.5, 1.5, 7.0, 6.0, 4.0).almostEquals(obj.getCentroidOfBounds(), 0.00000001));
  }

  /**
   * Test bounding box transform
   * @throws Exception
   */
  @Test
  public void testTransform() {

    // Test transform with no rotation
    List<Point3D> points = new LinkedList<>();
    points.add(new Point3D(-1, -1, -1));
    points.add(new Point3D(1, -1, -1));
    points.add(new Point3D(1, 1, -1));
    points.add(new Point3D(-1, 1, -1));
    points.add(new Point3D(-1, -1, 1));
    points.add(new Point3D(1, -1, 1));
    points.add(new Point3D(1, 1, 1));
    points.add(new Point3D(-1, 1, 1));
    
    CartesianObject cartObj = new CartesianObject(points);

    Transform frameToObj = new Transform(new Vector3(2, 2, 0), Quaternion.identity());

    CartesianObject resultObj = cartObj.transform(frameToObj);
    double[][] bounds = resultObj.getBounds();
    assertEquals(bounds[0][CartesianObject.MIN_BOUND_IDX], 1.0, 0.0000001);
    assertEquals(bounds[1][CartesianObject.MIN_BOUND_IDX], 1.0, 0.0000001);
    assertEquals(bounds[2][CartesianObject.MIN_BOUND_IDX], -1.0, 0.0000001);

    assertEquals(bounds[0][CartesianObject.MAX_BOUND_IDX], 3.0, 0.0000001);
    assertEquals(bounds[1][CartesianObject.MAX_BOUND_IDX], 3.0, 0.0000001);
    assertEquals(bounds[2][CartesianObject.MAX_BOUND_IDX], 1.0, 0.0000001);

    points = new LinkedList<>();
    points.add(new Point3D(-1, -1, -1));
    points.add(new Point3D(1, -1, -1));
    points.add(new Point3D(1, 1, -1));
    points.add(new Point3D(-1, 1, -1));
    points.add(new Point3D(-1, -1, 1));
    points.add(new Point3D(1, -1, 1));
    points.add(new Point3D(1, 1, 1));
    points.add(new Point3D(-1, 1, 1));

    // Test transform with 90 deg rotation around Z axis
    frameToObj = new Transform(new Vector3(2, 2, 0), Quaternion.fromAxisAngle(new Vector3(0, 0, 1), Math.toRadians(90)));

    resultObj = cartObj.transform(frameToObj);
    bounds = resultObj.getBounds();
    assertEquals(bounds[0][CartesianObject.MIN_BOUND_IDX], 1.0, 0.0000001);
    assertEquals(bounds[1][CartesianObject.MIN_BOUND_IDX], 1.0, 0.0000001);
    assertEquals(bounds[2][CartesianObject.MIN_BOUND_IDX], -1.0, 0.0000001);


    assertEquals(bounds[1][CartesianObject.MAX_BOUND_IDX], 3.0, 0.0000001);
    assertEquals(bounds[1][CartesianObject.MAX_BOUND_IDX], 3.0, 0.0000001);
    assertEquals(bounds[2][CartesianObject.MAX_BOUND_IDX], 1.0, 0.0000001);
  }

}
