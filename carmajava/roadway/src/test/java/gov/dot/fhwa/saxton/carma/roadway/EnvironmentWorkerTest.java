/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.HeadingStamped;
import cav_msgs.SystemAlert;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import nav_msgs.Odometry;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;
import tf2_msgs.TFMessage;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Runs unit tests for the RouteWorker class
 */
public class EnvironmentWorkerTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(EnvironmentWorkerTest.class);
    log.info("Setting up tests for RoadwayWorker");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Test the handling of system alert messages
   * @throws Exception
   */
  @Test
  public void testHandleSystemAlert() {

    // Check FATAL message
    MockEnvironmentManager envMgr = new MockEnvironmentManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(envMgr, log);
    SystemAlert alertMsg = messageFactory.newFromType(SystemAlert._TYPE);
    alertMsg.setType(SystemAlert.FATAL);

    assertTrue(!envMgr.isShutdown());
    envWkr.handleSystemAlertMsg(alertMsg);
    assertTrue(envMgr.isShutdown());

    // Check SHUTDOWN message
    envMgr = new MockEnvironmentManager();
    envWkr = new EnvironmentWorker(envMgr, log);
    alertMsg = messageFactory.newFromType(SystemAlert._TYPE);
    alertMsg.setType(SystemAlert.SHUTDOWN);

    assertTrue(!envMgr.isShutdown());
    envWkr.handleSystemAlertMsg(alertMsg);
    assertTrue(envMgr.isShutdown());
  }

  /**
   * Tests the handleHeadingMsg function
   * @throws Exception
   */
  @Test
  public void testHandleHeadingMsg() throws Exception {
    MockEnvironmentManager envMgr = new MockEnvironmentManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(envMgr, log);

    assertTrue(!envWkr.headingRecieved);
    HeadingStamped headingMsg = messageFactory.newFromType(HeadingStamped._TYPE);
    float heading = 135;
    headingMsg.setHeading(heading);
    envWkr.handleHeadingMsg(headingMsg);
    assertEquals(heading, envWkr.hostVehicleHeading, 0.0000000001);
    assertTrue(envWkr.headingRecieved);
  }

  /**
   * Tests the handleOdometryMsg function which is responsible for updating of the odom->base_link transform
   * @throws Exception
   */
  @Test
  public void testhandleOdometryMsg() throws Exception {
    MockEnvironmentManager envMgr = new MockEnvironmentManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(envMgr, log);
    // Build odometry message
    Odometry odometryMsg = messageFactory.newFromType(Odometry._TYPE);
    PoseWithCovariance pose = odometryMsg.getPose();
    pose.getPose().getPosition().setX(1);
    pose.getPose().getPosition().setY(2);
    pose.getPose().getPosition().setZ(0);
    Vector3 zAxis = new Vector3(0,0,1);
    double angle = Math.toRadians(45);
    Quaternion quat = Quaternion.fromAxisAngle(zAxis, angle);
    geometry_msgs.Quaternion quatMsg = pose.getPose().getOrientation();
    quatMsg = quat.toQuaternionMessage(quatMsg);
    pose.getPose().setOrientation(quatMsg);
    odometryMsg.setPose(pose);
    // Call function
    envWkr.handleOdometryMsg(odometryMsg);
    // Request transform from envMgr. It should be the same as what was passed in the odometry message
    Transform resultFromTF = envMgr.getTransform(envWkr.odomFrame, envWkr.baseLinkFrame);
    Vector3 resultFromTFTrans = resultFromTF.getTranslation();
    Quaternion resultFromTFRot = resultFromTF.getRotationAndScale();

    assertTrue(resultFromTFTrans.almostEquals(new Vector3(1,2,0), 0.0000001));
    assertTrue(resultFromTFRot.almostEquals(quat, 0.0001));
  }

  /**
   * Tests the handling of nav sat fix messages
   * Also tests the updating of the earth->map transform and map->odom transform
   * This test is dependant on the handleHeading and handleOdometry tests passing
   * @throws Exception
   */
  @Test
  public void testUpdatingOdomToBaseLink() throws Exception {
    MockEnvironmentManager envMgr = new MockEnvironmentManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(envMgr, log);

    // Publish the transform from base_link to position sensor
    Transform baseToPositionSensor = Transform.identity();
    TFMessage tfMsg = messageFactory.newFromType(TFMessage._TYPE);
    geometry_msgs.Transform baseToPositionSensorMsg = messageFactory.newFromType(geometry_msgs.Transform._TYPE);
    baseToPositionSensorMsg = baseToPositionSensor.toTransformMessage(baseToPositionSensorMsg);
    geometry_msgs.TransformStamped tfStamped = messageFactory.newFromType(TransformStamped._TYPE);
    tfStamped.setChildFrameId(envWkr.positionSensorFrame);
    tfStamped.getHeader().setFrameId(envWkr.baseLinkFrame);
    tfStamped.setTransform(baseToPositionSensorMsg);
    tfMsg.setTransforms(Arrays.asList(tfStamped));
    envMgr.publishTF(tfMsg);

    // Initial heading message
    HeadingStamped headingMsg = messageFactory.newFromType(HeadingStamped._TYPE);
    float heading = 90;
    headingMsg.setHeading(heading);
    envWkr.handleHeadingMsg(headingMsg);
    // Location at prime meridian and equator
    NavSatFix navMsg = messageFactory.newFromType(NavSatFix._TYPE);
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(0);
    navMsg.setLongitude(0);
    navMsg.setAltitude(0);
    envWkr.handleNavSatFixMsg(navMsg);

    assertTrue(envWkr.navSatFixRecieved);
    assertTrue(envWkr.hostVehicleLocation.almostEqual(new Location(0,0,0), 0.00001, 0.0000001));

    // First time calling nav sat fix so check earth to map transform (should be an NED frame at starting location)
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    Transform earthToMap = gcc.ecefToNEDFromLocaton(envWkr.hostVehicleLocation);
    // an NED at lat = 0 lon = 0 is rotated -90 deg around the ecef y-axis
    Vector3 solutionTrans = new Vector3(6378137.0, 0, 0);
    Vector3 solRotAxis = new Vector3(0,1,0);
    Quaternion solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(-90));
    assertTrue(earthToMap.getTranslation().almostEquals(solutionTrans, 1.0));// Check accuracy to within 1m
    assertTrue(earthToMap.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

    // The heading of 90 degrees means
    // the actual orientation of odom when there has been no odometry is with +x being due east
    solutionTrans = new Vector3(0.0, 0, 0);
    solRotAxis = new Vector3(0,0,1);
    solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(90));
    assertTrue(envWkr.mapToOdom.getTranslation().almostEquals(solutionTrans, 0.00001));
    assertTrue(envWkr.mapToOdom.getRotationAndScale().almostEquals(solutionRot, 0.0001));

    // Odometry moves the vehicle 10 meters along north axis
    // Build odometry message
    Odometry odometryMsg = messageFactory.newFromType(Odometry._TYPE);
    PoseWithCovariance pose = odometryMsg.getPose();
    pose.getPose().getPosition().setX(10); // 10 m north
    pose.getPose().getPosition().setY(0);
    pose.getPose().getPosition().setZ(0);
    geometry_msgs.Quaternion quatMsg = pose.getPose().getOrientation();
    pose.getPose().setOrientation(Quaternion.identity().toQuaternionMessage(quatMsg));

    // Call function
    envWkr.handleOdometryMsg(odometryMsg);

    // Find the new lat lon which will be used to compare
    Point3D realLocInMap = new Point3D(1,10,0);
    Location realLoc = gcc.cartesian2Geodesic(realLocInMap, earthToMap);
    // Send new lat lon to EnvironmentWorker
    navMsg = messageFactory.newFromType(NavSatFix._TYPE);
    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
    navMsg.setLatitude(realLoc.getLatitude());
    navMsg.setLongitude(realLoc.getLongitude());
    navMsg.setAltitude(realLoc.getAltitude());
    envWkr.handleNavSatFixMsg(navMsg);

    // Map earth transform should be unchanged
    assertTrue(earthToMap.almostEquals(envWkr.earthToMap, 0.0000001));
    // Compare new mapToOdom
    // The odom frame should be moved +1 along the map's +x axis
    solutionTrans = new Vector3(1, 0, 0);
    solRotAxis = new Vector3(0,0,1);
    solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(90));
    assertTrue(envWkr.mapToOdom.getTranslation().almostEquals(solutionTrans, 0.00001));
    assertTrue(envWkr.mapToOdom.getRotationAndScale().almostEquals(solutionRot, 0.0001));

    // Update earth map
    // Wait for enough time to pass for new map to be requested
    Transform oldEarthToMap = envWkr.earthToMap;
    Thread.sleep(envWkr.MAP_UPDATE_PERIOD.totalNsecs() / 1000000);
    // No new movement is needed
    envWkr.handleNavSatFixMsg(navMsg);
    // Find what new earth to map should be
    earthToMap = gcc.ecefToNEDFromLocaton(envWkr.hostVehicleLocation);
    assertTrue(!earthToMap.almostEquals(oldEarthToMap, 0.00000001));
    assertTrue(earthToMap.almostEquals(envWkr.earthToMap, 0.00000001));
    // Odom should now be located at -1,-10,0
    solutionTrans = new Vector3(0, -10, 0);
    solRotAxis = new Vector3(0,0,1);
    solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(90));
    assertTrue(envWkr.mapToOdom.getTranslation().almostEquals(solutionTrans, 0.00001));
    assertTrue(envWkr.mapToOdom.getRotationAndScale().almostEquals(solutionRot, 0.0001));
  }
//  /**
//   * Tests the detection of a vehicle leaving a route
//   * @throws Exception
//   */
//  @Test
//  public void testLeavingRouteVicinity() throws Exception {
//    RouteWorker routeWorker = new RouteWorker(new MockRouteManager(), log, "src/test/resources/routefiles/");
//
//    routeWorker.handleSystemAlertMsg(routeWorker.buildSystemAlertMsg(SystemAlert.DRIVERS_READY, ""));
//    assertTrue(routeWorker.systemOkay == true);
//
//    // Test vehicle has been placed just before start of route
//    NavSatFix navMsg = messageFactory.newFromType(NavSatFix._TYPE);
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.95649);
//    navMsg.setLongitude(-77.15028);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.getCurrentState() == WorkerState.ROUTE_SELECTION);
//
//    byte response = routeWorker.setActiveRoute("Colonial Farm Rd. Outbound");
//    assertTrue(response == SetActiveRouteResponse.NO_ERROR);
//    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);
//
//    response = routeWorker.startActiveRoute();
//    assertTrue(response == StartActiveRouteResponse.NO_ERROR);
//
//    assertTrue(routeWorker.currentSegmentIndex == 0); // start of route
//
//    assertTrue(routeWorker.currentSegment.getUptrackWaypoint().getLocation().almostEqual(
//      new Location(38.95649, -77.15028, 0), 0.0000001, 0.0000001));
//    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);
//
//    // Test vehicle has been placed after route file starting point
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.9564);
//    navMsg.setLongitude(-77.15035);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
//    assertTrue(routeWorker.getCurrentState() == WorkerState.FOLLOWING_ROUTE);
//
//    // Test vehicle has been placed inside the garage (off the route)
//    navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//    navMsg.setLatitude(38.95633);
//    navMsg.setLongitude(-77.15011);
//    navMsg.setAltitude(0);
//    routeWorker.handleNavSatFixMsg(navMsg);
//
//    assertTrue(routeWorker.currentSegmentIndex == 1); // second segment
//    assertTrue(routeWorker.leftRouteVicinity());
//    assertTrue(routeWorker.getCurrentState() == WorkerState.WAITING_TO_START);
//  }
}
