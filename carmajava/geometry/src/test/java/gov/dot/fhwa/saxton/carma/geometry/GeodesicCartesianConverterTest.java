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

import geometry_msgs.TransformStamped;
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
 * Runs unit tests for the GeodesicCartesianConverter class
 */
public class GeodesicCartesianConverterTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(GeodesicCartesianConverterTest.class);
    log.info("Setting up tests for GeodesicCartesianConverter");
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
    GeodesicCartesianConverter gcConvertor = new GeodesicCartesianConverter();

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
// TODO NEW -----
    // Test altitude change
    point = new Point3D(0, 0, 0);
    // The location of the ecef frame in the desired frame
    tf = new Transform(new Vector3(0,0,6356782), Quaternion.identity());
    loc = gcConvertor.cartesian2Geodesic(point, tf);
    solution = new Location(90, 0, 30);
    assertTrue(loc.almostEqual(solution, 0.0001, 1.0)); // Check accuracy to within about 1m


    Transform hostInBaseLink = new Transform(new Vector3(1.524, 0.0, -0.3556), new Quaternion(1.0, 0.0, 0.0, -1.03411553555e-13));

    Transform mapInEarth = new Transform(new Vector3(1178560.13006, -4786930.86843, 4033296.49897), new Quaternion(0.55783250982, 0.711831079304, 0.263230968008, -0.335900078903));

    Transform odomInMap = new Transform(new Vector3(1195.83171134, -874.162787034, 7.90023773302), new Quaternion(0.0103026125862, 0.013543380725, -0.89042302604, -0.454815641453));

    Transform baseLinkInOdom = new Transform(new Vector3(1344.19604492, 573.833007812, 18.4290008546), new Quaternion(0.521438406167, 0.853121326139, 0.0157684293929, -0.00611130880506));

    Transform hostInEarth = mapInEarth.multiply(odomInMap).multiply(baseLinkInOdom).multiply(hostInBaseLink);

    System.out.println("\n\n\n HostInEarth = " + hostInEarth + "\n");
    Vector3 vec = hostInEarth.getTranslation();
    Point3D p = new Point3D(vec.getX(), vec.getY(), vec.getZ());
    System.out.println(gcConvertor.cartesian2Geodesic(p, Transform.identity()));
    System.out.println(gcConvertor.cartesian2Geodesic(new Point3D(0,0,0), hostInEarth));
    System.out.println("\n\n");

// TODO end NEW ----
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
    GeodesicCartesianConverter gcConvertor = new GeodesicCartesianConverter();

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

  /**
   * Tests the ecefToNEDFromLocaton frunction
   * @throws Exception
   */
  @Test
  public void testEcefToNEDFromLocaton() throws Exception {
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();

    // Prime meridian and equator
    Location locOfNED = new Location(0, 0, 0);
    Transform ecefToNED = gcc.ecefToNEDFromLocaton(locOfNED);
    Vector3 trans = ecefToNED.getTranslation();
    Quaternion rot = ecefToNED.getRotationAndScale();
    // an NED at lat = 0 lon = 0 is rotated -90 deg around the ecef y-axis
    Vector3 solutionTrans = new Vector3(6378137.0, 0, 0);
    Vector3 solRotAxis = new Vector3(0,1,0);
    Quaternion solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(-90));
    assertTrue(trans.almostEquals(solutionTrans, 1.0));// Check accuracy to within 1m
    assertTrue(rot.almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

    // Equator at intersection of ECEF y-axis
    locOfNED = new Location(0, 90, 0);
    ecefToNED = gcc.ecefToNEDFromLocaton(locOfNED);
    trans = ecefToNED.getTranslation();
    rot = ecefToNED.getRotationAndScale();
    // an NED at lat = 0 lon = 90 is rotated -90 deg around the ecef y-axis then +90 around the new x-axis
    // Equivalent quaternion found with http://www.andre-gaschler.com/rotationconverter/
    solutionTrans = new Vector3(0, 6378137.0, 0);
    solutionRot = new Quaternion(0.5, -0.5, 0.5, 0.5);
    assertTrue(trans.almostEquals(solutionTrans, 1.0));// Check accuracy to within 1m
    assertTrue(rot.almostEquals(solutionRot.normalize(), 0.0001)); // Check accuracy to within ~0.01 deg

    // Prime meridian at 45 deg lat
    locOfNED = new Location(45, 0, 0);
    ecefToNED = gcc.ecefToNEDFromLocaton(locOfNED);
    trans = ecefToNED.getTranslation();
    rot = ecefToNED.getRotationAndScale();
    // an NED at lat = 45 lon = 0 is rotated -135 deg around the ecef y-axis
    Point3D locInECEF = gcc.geodesic2Cartesian(locOfNED, Transform.identity());
    solutionTrans = new Vector3(locInECEF.getX(), locInECEF.getY(), locInECEF.getZ());
    solRotAxis = new Vector3(0,1,0);
    solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(225));
    assertTrue(trans.almostEquals(solutionTrans, 1.0));// Check accuracy to within 1m
    assertTrue(rot.almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

  }

}
