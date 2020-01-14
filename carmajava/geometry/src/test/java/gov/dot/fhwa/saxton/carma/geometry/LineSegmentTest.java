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
import org.ros.rosjava_geometry.Transform;

import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Created by mcconnelms on 9/25/17.
 */
public class LineSegmentTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(LineSegment.class);
    log.info("Setting up tests for LineSegment");
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
    Point p1 = new Point3D(1,0,0);
    Point p2 = new Point3D(0,0,0);
    LineSegment line = new LineSegment(p1, p2);
    assertEquals(3, line.getNumDimensions());
    assertEquals(p1.distanceFrom(p2),line.length(), 0.0000001);

    // Deep copy constructor
    LineSegment line2 = new LineSegment(line);
    assertTrue(line2 != line);
    assertEquals(line.getNumDimensions(), line2.getNumDimensions());
    assertEquals(line.length(), line2.length(), 0.0000001);
  }

  /**
   * Tests the perpendicularDistance function
   * @throws Exception
   */
  @Test
  public void testPerpendicularDistance() throws Exception {
    Point p1 = new Point2D(1,1);
    Point p2 = new Point2D(3,1);
    Point p3 = new Point2D(2,2);

    // Simple test
    LineSegment line = new LineSegment(p1, p2);
    assertEquals(1.0, line.perpendicularDistance(p3), 0.0000001);

    // Test inline point
    p3 = new Point2D(5, 1);
    assertEquals(0.0, line.perpendicularDistance(p3), 0.0000001);

    // Test comparison of different dimension points
    p3 = new Point3D(0,0,0);
    try {
      line.perpendicularDistance(p3);
      fail("Expected exception was not thrown when using non-equal dimensions");
    } catch (IllegalArgumentException e) {
      assertTrue("Caught expected exception", 1==1);
    }

    // Test 3D space with negative values
    p1 = new Point3D(-1,-1,-1);
    p2 = new Point3D(-3,-1,-1);
    p3 = new Point3D(4,-2,-2);

    line = new LineSegment(p1, p2);
    assertEquals(1.4142135, line.perpendicularDistance(p3), 0.0000001);

  }

  /**
   * Tests the pointProjectedOnLine function
   * @throws Exception
   */
  @Test
  public void testPointProjectedOnLine() throws Exception {
    Point p1 = new Point2D(1,1);
    Point p2 = new Point2D(3,1);
    Point p3 = new Point2D(2,2);

    // Simple test
    LineSegment line = new LineSegment(p1, p2);
    Point solution = new Point2D(2,1);
    assertEquals(0.0, line.pointProjectedOnLine(p3).distanceFrom(solution), 0.0000001);

    // Test inline point
    p3 = new Point2D(5, 1);
    solution = p3;
    assertEquals(0.0, line.pointProjectedOnLine(p3).distanceFrom(solution), 0.0000001);

    // Test comparison of different dimension points
    p3 = new Point3D(0,0,0);
    try {
      line.pointProjectedOnLine(p3);
      fail("Expected exception was not thrown when using non-equal dimensions");
    } catch (IllegalArgumentException e) {
      assertTrue("Caught expected exception", 1==1);
    }
  }

    /**
   * Tests the crossTrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testCrossTrackDistance() throws Exception {

    log.info("// Entering crossTrackDistance test");

    // Test on km long segment (Right side)
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    // Test on km long segment
    Point3D loc1 = gcc.geodesic2Cartesian(new Location(38.942201,-77.160108, 0), Transform.identity());
    Point3D loc2 = gcc.geodesic2Cartesian(new Location(38.943804, -77.148832, 0), Transform.identity());
    LineSegment3D seg = new LineSegment3D(loc1, loc2);
    Point3D sideLoc = gcc.geodesic2Cartesian(new Location(38.942422, -77.154786, 0), Transform.identity());
    double solution = 58.59;
    assertEquals(seg.crossTrackDistance(sideLoc), solution, 0.1); // Check accuracy to within .1m

    // Test on km long segment (Left side)
    sideLoc = gcc.geodesic2Cartesian(new Location(38.94348, -77.15505, 0), Transform.identity());
    solution = -61.14;
    assertEquals(seg.crossTrackDistance(sideLoc), solution, 0.1); // Check accuracy to within .1m

    // Test point on segment start
    solution = 0.0;
    assertEquals(seg.crossTrackDistance(loc1), solution, 0.1); // Check accuracy to within .1m

    // Test point on segment start
    solution = 0.0;
    assertEquals(seg.crossTrackDistance(loc1), solution, 0.1); // Check accuracy to within .1m
  }

  /**
   * Tests the downtrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testDownTrackDistance() throws Exception {

    log.info("// Entering downtrackDistance test");
    
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    // Test on km long segment
    Point3D loc1 = gcc.geodesic2Cartesian(new Location(38.942201,-77.160108, 0), Transform.identity());
    Point3D loc2 = gcc.geodesic2Cartesian(new Location(38.943804, -77.148832, 0), Transform.identity());
    LineSegment3D seg = new LineSegment3D(loc1, loc2);
    Point3D sideLoc = gcc.geodesic2Cartesian(new Location(38.942422, -77.154786, 0), Transform.identity());
    double solution = 458.3;
    assertEquals(seg.downtrackDistance(sideLoc), solution, 0.1); // Check accuracy to within .1m

    // Test point on segment start
    solution = 0.0;
    assertEquals(seg.downtrackDistance(loc1), solution, 0.1); // Check accuracy to within .1m

    // Test point on segment end
    solution = 991.4;
    assertEquals(seg.downtrackDistance(loc2), solution, solution * 0.005); // Check accuracy to within .5% of result
  }
}
