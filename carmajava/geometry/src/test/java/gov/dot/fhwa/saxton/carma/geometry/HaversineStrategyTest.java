package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.GreatCircleSegment;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.HaversineStrategy;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import static org.junit.Assert.assertTrue;

/**
 * Runs unit tests for the GeodesicCartesianConvertor class
 */
public class HaversineStrategyTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(HaversineStrategyTest.class);
    log.info("Setting up tests for HaversineStrategy");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the distanceLoc2Loc function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testDistanceLoc2Loc() throws Exception {

    log.info("// Entering distanceLoc2Loc test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test points on example garage to colonial farm rd route
    Location loc1 = new Location(38.95647,-77.15031, 0);
    Location loc2 = new Location(38.95631, -77.15041, 0);
    double solution = 19.78;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m

    loc1 = new Location(38.95628,-77.15047, 0);
    loc2 = new Location(38.95613, -77.15101, 0);
    solution = 49.58;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m

    // Test points about a km apart
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    solution = 991.4;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m
  }

  /**
   * Tests the crossTrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testCrossTrackDistance() throws Exception {

    log.info("// Entering crossTrackDistance test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test on km long segment
    Location loc1 = new Location(38.942201,-77.160108, 0);
    Location loc2 = new Location(38.943804, -77.148832, 0);
    GreatCircleSegment seg = new GreatCircleSegment(loc1, loc2);
    Location sideLoc = new Location(38.942422, -77.154786, 0);
    double solution = 58.59;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(sideLoc, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment start
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(loc1, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment end
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(loc2, seg) - solution) < 0.1); // Check accuracy to within .1m
  }

  /**
   * Tests the downtrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testDownTrackDistance() throws Exception {

    log.info("// Entering downtrackDistance test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test on km long segment
    Location loc1 = new Location(38.942201,-77.160108, 0);
    Location loc2 = new Location(38.943804, -77.148832, 0);
    GreatCircleSegment seg = new GreatCircleSegment(loc1, loc2);
    Location sideLoc = new Location(38.942422, -77.154786, 0);
    double solution = 457.1;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(sideLoc, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment start
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(loc1, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment end
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 991.4;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(loc2, seg) - solution) < 0.1); // Check accuracy to within .1m
  }
}
