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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Unit tests for the Point class
 */
public class PointTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(PointTest.class);
    log.info("Setting up tests for Point");
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
    // Array Constructor
    double[] dims = { 4, 5, 6, 7 };
    Point p = new Point(dims);
    assertTrue(p.getNumDimensions() == 4);
    assertTrue(p.getDim(0) == 4);
    assertTrue(p.getDim(3) == 7);

    // Deep copy constructor
    dims = new double[] { 1, 2 };
    Point p2 = new Point(dims);
    p = new Point(new Point(dims));
    assertTrue(p != p2);
    assertTrue(p.getDim(0) == 1);
    assertTrue(p.getDim(1) == 2);
  }

  /**
   * Tests the getNumDimensions function
   * @throws Exception
   */
  @Test
  public void testGetNumDimensions() throws Exception {
    double[] dims = { 4, 5, 6, 7 };
    Point p = new Point(dims);
    assertTrue(p.getNumDimensions() == 4);

    dims = new double[]{};
    p = new Point(dims);
    assertTrue(p.getNumDimensions() == 0);
  }

  /**
   * Tests the setting and getting of dimensions
   * @throws Exception
   */
  @Test
  public void testGetSetDim() throws Exception {
    double[] dims = { 4, 5, 6, 7 };
    Point p = new Point(dims);
    try {
      assertTrue(p.getDim(0) == 4);
    } catch (IndexOutOfBoundsException e) {
      fail("Invalid index in testGetSetDim");
    }

    double a = 5;
    try {
      a = p.getDim(-1);
      fail("Invalid dimension set without exception");
    } catch (IndexOutOfBoundsException e) {
      assertTrue(a == 5);
    }

    try {
      a = p.getDim(4);
      fail("Invalid dimension set without exception");
    } catch (IndexOutOfBoundsException e) {
      assertTrue(a == 5);
    }

  }

  /**
   * Tests the distance from function
   * @throws Exception
   */
  @Test
  public void testDistanceFrom() throws Exception {
    // Simple test
    double[] dims = {0,0,0};
    double[] dims2 = {1,0,0};
    Point p = new Point(dims);
    Point p2 = new Point(dims2);
    assertTrue(Math.abs(p.distanceFrom(p2) - 1.0) < 0.00000001);

    // With negatives
    dims2 = new double[]{-1,0,0};
    p2 = new Point(dims2);
    assertTrue(Math.abs(p.distanceFrom(p2) - 1.0) < 0.00000001);

    // Same location
    dims2 = new double[]{0,0,0};
    p2 = new Point(dims2);
    assertTrue(Math.abs(p.distanceFrom(p2) - 0.0) < 0.00000001);

    // 2D test
    p = new Point(0,0);
    p2 = new Point(0,1);
    assertTrue(Math.abs(p.distanceFrom(p2) - 1.0) < 0.00000001);

    // Complicated distance calc
    dims = new double[]{7,4,3};
    dims2 = new double[]{17,6,2};
    p = new Point(dims);
    p2 = new Point(dims2);
    assertTrue(Math.abs(p.distanceFrom(p2) - 10.246951) < 0.000001);

    // Mismatched dimensions test
    dims = new double[]{1};
    dims2 = new double[]{5,7,9};
    p = new Point(dims);
    p2 = new Point(dims2);
    try {
      p.distanceFrom(p2);
      fail("Exception for mismatched point dimensions not thrown");
    } catch (IllegalArgumentException e) {
      assertTrue(1==1);
    }
  }

  /**
   * Tests the almostEquals function
   * @throws Exception
   */
  @Test
  public void testAlmostEquals() throws Exception {
    // Equal
    double[] dims = {0,0,0};
    double[] dims2 = {0,0,0};
    Point p = new Point(dims);
    Point p2 = new Point(dims2);
    assertTrue(p.almostEquals(p2,0.00000001));

    // Not equal
    dims2 = new double[]{1,0,0};
    p2 = new Point(dims2);
    assertTrue(!p.almostEquals(p2,0.00000001));

    // Large delta
    dims2 = new double[]{1,0,0};
    p2 = new Point(dims2);
    assertTrue(p.almostEquals(p2,1.1));
  }
}
