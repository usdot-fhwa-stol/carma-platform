package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.*;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import javax.sound.sampled.Line;

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
}
