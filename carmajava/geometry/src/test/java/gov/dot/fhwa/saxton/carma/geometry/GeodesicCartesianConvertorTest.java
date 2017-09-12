package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import static org.junit.Assert.*;

/**
 * Runs unit tests for the GeodesicCartesianConvertor class
 */
public class GeodesicCartesianConvertorTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(GeodesicCartesianConvertorTest.class);
    log.info("Setting up tests for GeodesicCartesianConvertor");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the cartesian2Geodesic function
   * @throws Exception
   */
  @Test
  public void testCartesian2Geodesic() throws Exception {
    // ECEF frame points computed using online calculator http://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    log.info("// Entering cartesian2Geodesic test");
    GeodesicCartesianConvertor gcConvertor = new GeodesicCartesianConvertor();

    // Test point nearish TFHRC
    Point3D point = new Point3D(1104488, -4841993, 3988562);
    // The location of the ecef frame in the desired frame
    Transform tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    Location loc = gcConvertor.cartesian2Geodesic(point, tf);
    Location solution = new Location(38.956488, -77.150345, 0);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m

    // Test equator and prime meridian
    point = new Point3D(6378137, 0, 0);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(0, 0, 0);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m

    // Test north pole
    point = new Point3D(0, 0, 6356752);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(90, 0, 0);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m

    // Test south pole
    point = new Point3D(0, 0, -6356752);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(-90, 0, 0);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m

    // Test altitude change
    point = new Point3D(0, 0, 6356782);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(90, 0, 30);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m

    // Test complex transform with frame located at north pole rotated 90 degrees around ecef z axis
    // The point being transformed is located at the equator/prime meridian
    point = new Point3D(0, -6378137, -6356752);
    // Rotation 90 degrees around z axis of ecef frame
    Quaternion quat = Quaternion.fromAxisAngle(new Vector3(0,0,1), Math.PI/2.0);
    // The location of the ecef frame in the desired frame (at north pole)
    tf = new Transform(new Vector3(0,0,6356752), quat);
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(0, 0, 0);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m
  }

  /**
   * Tests the geodesic2Cartesian function
   * @throws Exception
   */
  @Test
  public void testGeodesic2Cartesian() throws Exception {
    // ECEF frame points computed using online calculator http://www.oc.nps.edu/oc2902w/coord/llhxyz.htm

    log.info("// Entering geodesic2Cartesian test");
    GeodesicCartesianConvertor gcConvertor = new GeodesicCartesianConvertor();

    // Test point near STOL garage at TFHRC
    Location loc = new Location(38.956488, -77.150345, 0);
    // The location of the ecef frame in the desired frame
    Transform tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    Point3D point = gcConvertor.geodesic2Cartesian(loc, tf);
    Point3D solution = new Point3D(1104488, -4841993, 3988562);
    assertTrue(point.almostEquals(solution, 1)); // Check accuracy to within 1m

    // Test equator and prime meridian
    loc = new Location(0, 0, 0);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    point = gcConvertor.geodesic2Cartesian(loc, tf);
    solution = new Point3D(6378137, 0, 0);
    assertTrue(point.almostEquals(solution, 1)); // Check accuracy to within 1m

    // Test north pole
    loc = new Location(90, 0, 0);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    point = gcConvertor.geodesic2Cartesian(loc, tf);
    solution = new Point3D(0, 0, 6356752);
    assertTrue(point.almostEquals(solution, 1)); // Check accuracy to within 1m

    // Test altitude change
    loc = new Location(90, 0, 30);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,0), Quaternion.identity());
    point = gcConvertor.geodesic2Cartesian(loc, tf);
    solution = new Point3D(0, 0, 6356782);
    assertTrue(point.almostEquals(solution, 1)); // Check accuracy to within 1m

    // Test complex transform with frame located at north pole rotated 90 degrees around ecef z axis
    // The point being transformed is located at the equator/prime meridian
    loc = new Location(0, 0, 0);
    // Rotation -90 degrees around z axis of desired frame (at north pole)
    Quaternion quat = Quaternion.fromAxisAngle(new Vector3(0,0,1), -Math.PI/2.0);
    // The location of the ecef frame in the desired frame (at north pole)
    tf = new Transform(new Vector3(0,0,-6356752), quat);
    point = gcConvertor.geodesic2Cartesian(loc, tf);
    solution = new Point3D(0, -6378137, -6356752);
    assertTrue(point.almostEquals(solution, 1)); // Check accuracy to within 1m
  }

}
