/*
 * Copyright (C) 2017 LEIDOS.
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

import cav_msgs.ExternalObject;
import cav_msgs.HeadingStamped;
import cav_msgs.SystemAlert;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import nav_msgs.Odometry;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
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
 * Runs unit tests for the TransformMaintainer class
 */
public class EnvironmentWorkerTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(EnvironmentWorker.class);
    log.info("Setting up tests for EnvironmentWorker");
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
    MockRoadwayManager roadwayMgr = new MockRoadwayManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(roadwayMgr, log, "earth", "map", "odom",
      "base_link", "pinpoint", "pinpoint", 200, 100);
    SystemAlert alertMsg = messageFactory.newFromType(SystemAlert._TYPE);
    alertMsg.setType(SystemAlert.FATAL);

    assertTrue(!roadwayMgr.isShutdown());
    envWkr.handleSystemAlertMsg(alertMsg);
    assertTrue(roadwayMgr.isShutdown());

    // Check SHUTDOWN message
    roadwayMgr = new MockRoadwayManager();
    envWkr = new EnvironmentWorker(roadwayMgr, log, "earth", "map", "odom",
      "base_link", "pinpoint", "pinpoint", 200, 100);
    alertMsg = messageFactory.newFromType(SystemAlert._TYPE);
    alertMsg.setType(SystemAlert.SHUTDOWN);

    assertTrue(!roadwayMgr.isShutdown());
    envWkr.handleSystemAlertMsg(alertMsg);
    assertTrue(roadwayMgr.isShutdown());
  }

  /**
   * Test bounding box conversion
   * @throws Exception
   */
  @Test
  public void testGetAABB() {

    // Test transform with no rotation
    MockRoadwayManager roadwayMgr = new MockRoadwayManager();
    EnvironmentWorker envWkr = new EnvironmentWorker(roadwayMgr, log, "earth", "map", "odom",
      "base_link", "pinpoint", "pinpoint", 200, 100);
    ExternalObject obj = messageFactory.newFromType(ExternalObject._TYPE);
    obj.getSize().setX(1);
    obj.getSize().setY(1);
    obj.getSize().setZ(0);
    Transform frameToObj = new Transform(new Vector3(2, 2, 0), Quaternion.identity());

    Vector3D[] bounds = envWkr.getAABB(obj, frameToObj);
    assertEquals(bounds[0].getX(), 1.0, 0.0000001);
    assertEquals(bounds[0].getY(), 1.0, 0.0000001);
    assertEquals(bounds[0].getZ(), 0.0, 0.0000001);

    assertEquals(bounds[1].getX(), 3.0, 0.0000001);
    assertEquals(bounds[1].getY(), 3.0, 0.0000001);
    assertEquals(bounds[1].getZ(), 0.0, 0.0000001);

    // Test transform with 90 deg rotation around Z axis
    frameToObj = new Transform(new Vector3(2, 2, 0), Quaternion.fromAxisAngle(new Vector3(0, 0, 1), 90));

    bounds = envWkr.getAABB(obj, frameToObj);
    assertEquals(bounds[0].getX(), 1.0, 0.0000001);
    assertEquals(bounds[0].getY(), 1.0, 0.0000001);
    assertEquals(bounds[0].getZ(), 0.0, 0.0000001);

    assertEquals(bounds[1].getX(), 3.0, 0.0000001);
    assertEquals(bounds[1].getY(), 3.0, 0.0000001);
    assertEquals(bounds[1].getZ(), 0.0, 0.0000001);

  }
}
