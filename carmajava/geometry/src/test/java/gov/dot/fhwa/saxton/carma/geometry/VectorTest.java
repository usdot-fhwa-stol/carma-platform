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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Unit tests for the Vector class
 */
public class VectorTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(VectorTest.class);
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

    // Two point constructor
    Point head = new Point3D(1,0,0);
    Point tail = new Point3D(0,0,0);
    Vector v = new Vector(tail, head);
    assertTrue(v.getNumDimensions() == 3);
    assertTrue(Math.abs(v.getDim(0) - 1) < 0.0000001);

    tail = new Point(new double[]{1,1});
    try {
      Vector tmp = new Vector(tail, head);
    } catch (IllegalArgumentException e) {
      assertTrue("Caught exception as expected", 1 == 1);
    }

    // Single point constructor
    v = new Vector(head);
    assertTrue(v.getNumDimensions() == 3);
    assertTrue(Math.abs(v.getDim(0) - 1) < 0.0000001);

    // Deep copy constructor
    Vector v2 = new Vector(v);
    assertTrue(v2 != v);
    assertTrue(v2.getNumDimensions() == 3);
    assertTrue(Math.abs(v2.getDim(0) - 1) < 0.0000001);
  }

  /**
   * Tests the magnitude function
   * @throws Exception
   */
  @Test
  public void testMagnitude() throws Exception {
    Point head = new Point3D(1,0,0);
    Point tail = new Point3D(0,0,0);
    Vector v = new Vector(tail, head);
    assertTrue(Math.abs(v.magnitude() - 1.0) < 0.0000001);

    head = new Point3D(0,0,0);
    tail = new Point3D(0,0,0);
    v = new Vector(tail, head);
    assertTrue(Math.abs(v.magnitude() - 0.0) < 0.0000001);

    head = new Point2D(0,0);
    tail = new Point2D(3,4);
    v = new Vector(tail, head);
    assertTrue(Math.abs(v.magnitude() - 5.0) < 0.0000001);
  }

  /**
   * Tests the add function
   * @throws Exception
   */
  @Test
  public void testAdd() throws Exception {
    Point head = new Point3D(1,0,0);
    Vector v = new Vector(head);
    Vector v2 = new Vector(new Point3D(2,4,-1));
    v = v.add(v2);
    assertTrue(Math.abs(v.getDim(0) - 3.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(1) - 4.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(2) - -1.0) < 0.0000001);

    // Test dimension mismatch
    head = new Point3D(0,0,0);
    v = new Vector(head);
    v2 = new Vector(new Point2D(0,0));
    try {
      v = v.add(v2);
      fail("Exception for mismatched vector dimensions not thrown");
    } catch (IllegalArgumentException e) {
      //If we make it here the test has passed
    }
  }

  /**
   * Tests the subtract function
   * @throws Exception
   */
  @Test
  public void testSubtract() throws Exception {
    Point head = new Point3D(1,0,0);
    Vector v = new Vector(head);
    Vector v2 = new Vector(new Point3D(2,4,-1));
    v = v.subtract(v2);
    assertTrue(Math.abs(v.getDim(0) - -1.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(1) - -4.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(2) - 1.0) < 0.0000001);

    // Test dimension mismatch
    head = new Point3D(0,0,0);
    v = new Vector(head);
    v2 = new Vector(new Point2D(0,0));
    try {
      v = v.subtract(v2);
      fail("Exception for mismatched vector dimensions not thrown");
    } catch (IllegalArgumentException e) {
      // If we make it here the test has passed
    }
  }

  /**
   * Tests the scalarMultiply function
   * @throws Exception
   */
  @Test
  public void testScalarMultiply() throws Exception {
    Point head = new Point3D(1,-2.0,0);
    Vector v = new Vector(head);
    v = v.scalarMultiply(5);
    assertTrue(Math.abs(v.getDim(0) - 5.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(1) - -10.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(2) - 0.0) < 0.0000001);
  }

  /**
   * Tests the elementWiseMultiply function
   * @throws Exception
   */
  @Test
  public void testElementWiseMultiply() throws Exception {
    Point head = new Point3D(1,-2.0,0);
    Vector v = new Vector(head);
    Vector v2 = new Vector(new Point3D(2,3,4));
    v = v.elementWiseMultiply(v2);
    assertTrue(Math.abs(v.getDim(0) - 2.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(1) - -6.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(2) - 0.0) < 0.0000001);

    // Test dimension mismatch
    head = new Point3D(0,0,0);
    v = new Vector(head);
    v2 = new Vector(new Point2D(0,0));
    v = v.elementWiseMultiply(v2);
    assertTrue(Math.abs(v.getDim(0) - 0.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(1) - 0.0) < 0.0000001);
    assertTrue(Math.abs(v.getDim(2) - 0.0) < 0.0000001);
  }

  /**
   * Tests the dot function
   * @throws Exception
   */
  @Test
  public void testDotProduct() throws Exception {
    Point head = new Point3D(1,-2.0,0);
    Vector v = new Vector(head);
    Vector v2 = new Vector(new Point3D(2,3,4));
    assertTrue(Math.abs(v.dot(v2) - -4.0) < 0.0000001);

    head = new Point3D(0,0,0);
    v = new Vector(head);
    v2 = new Vector(new Point3D(0,0, 0));
    v = v.subtract(v2);
    assertTrue(Math.abs(v.dot(v2) - 0.0) < 0.0000001);

    // Test dimension mismatch
    try {
      Vector tmp = new Vector(new Point2D(0,0));
      v.dot(tmp);
      fail("Exception was not caught");
    } catch (IllegalArgumentException e) {
      assertTrue("Caught exception as expected", 1 == 1);
    }
  }

  /**
   * Tests the getUnitVector function
   * @throws Exception
   */
  @Test
  public void testGetUnitVector() throws Exception {
    Point head = new Point3D(9,9,9);
    Vector v = new Vector(head);
    Vector unitV = v.getUnitVector();
    assertTrue(Math.abs(unitV.getDim(0) - (1.0 / Math.sqrt(3.0))) < 0.0000001);
    assertTrue(Math.abs(unitV.getDim(1) - (1.0 / Math.sqrt(3.0))) < 0.0000001);
    assertTrue(Math.abs(unitV.getDim(2) - (1.0 / Math.sqrt(3.0))) < 0.0000001);
  }

  /**
   * Tests the getInteriorAngle function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testGetAngleBetweenVectors() throws Exception {

    log.info("// Entering get interior angle test");
    Vector3D v1 = new Vector3D(1,1,1);
    Vector3D v2 = new Vector3D(1,2,4);
    double angle = v1.getAngleBetweenVectors(v2);
    assertTrue(Math.abs(angle - 0.4908826) < 0.00001);

    v1 = new Vector3D(8,-2,13);
    v2 = new Vector3D(-2,14,-2);
    angle = v1.getAngleBetweenVectors(v2);
    assertTrue(Math.abs(angle - 1.89478) < 0.00001);

    v1 = new Vector3D(1,0,0);
    v2 = new Vector3D(0,1,0);
    angle = v1.getAngleBetweenVectors(v2);
    assertTrue(Math.abs(angle - Math.PI/2.0) < 0.00001);

    v1 = new Vector3D(1,0,0);
    v2 = new Vector3D(0,0,1);
    angle = v1.getAngleBetweenVectors(v2);
    assertTrue(Math.abs(angle - Math.PI/2.0) < 0.00001);

    v1 = new Vector3D(0,0,0);
    v2 = new Vector3D(0,0,0);
    angle = v1.getAngleBetweenVectors(v2);
    assertTrue(Math.abs(angle - 0.0) < 0.00001);
  }

  /**
   * Tests the cross product function
   * @throws Exception
   */
  @Test
  public void testCrossProduct() throws Exception {
    // Simple unit vector test
    Vector3D v = new Vector3D(new Point3D(1,0,0));
    Vector3D v2 = new Vector3D(new Point3D(0,1,0));
    assertEquals(0, v.cross(v2).getX(),0.0000001);
    assertEquals(0, v.cross(v2).getY(),0.0000001);
    assertEquals(1, v.cross(v2).getZ(),0.0000001);

    // Test with negatives
    v = new Vector3D(new Point3D(-4,5,7));
    v2 = new Vector3D(new Point3D(2,5,6));
    assertEquals(-5, v.cross(v2).getX(),0.0000001);
    assertEquals(38, v.cross(v2).getY(),0.0000001);
    assertEquals(-30, v.cross(v2).getZ(),0.0000001);

    // Test with decimals
    v = new Vector3D(new Point3D(1.45,5.23,0.5));
    v2 = new Vector3D(new Point3D(-4.3,0.2,5));
    assertEquals(26.05, v.cross(v2).getX(),0.0000001);
    assertEquals(-9.4, v.cross(v2).getY(),0.0000001);
    assertEquals(22.779, v.cross(v2).getZ(),0.0000001);

    // Test same vector
    v = new Vector3D(new Point3D(1,2,3));
    assertEquals(0, v.cross(v).getX(),0.0000001);
    assertEquals(0, v.cross(v).getY(),0.0000001);
    assertEquals(0, v.cross(v).getZ(),0.0000001);
  }

  /**
   * Tests the 3x3 determinant funcation
   * @throws Exception
   */
  @Test
  public void test3by3Determinant() throws Exception {
    Vector3D v1 = new Vector3D(6,4,2); //Each vector is a col of the matrix
    Vector3D v2 = new Vector3D(1,-2,8);
    Vector3D v3 = new Vector3D(1,5,7);
    assertEquals(-306.0, Vector3D.get3by3Determinant(v1,v2,v3), 0.0000001);
  }
}
