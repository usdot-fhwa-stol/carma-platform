/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.IntersectionData;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.RoutePointStamped;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.EadIntersectionManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.PlanInterpolator;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

import cav_msgs.*;
import static org.mockito.Mockito.*;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

/**
 * Runs unit tests for the RouteWorker class
 */
public class IntersectionProcessingTest {

  Log log;
  MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
  IGlidepathAppConfig mockConfig;

  @Before
  public void setUp() throws Exception {
    ILoggerFactory mockFact = mock(ILoggerFactory.class);
    ILogger mockLogger = mock(ILogger.class);
    when(mockFact.createLoggerForClass(anyObject())).thenReturn(mockLogger);
    LoggerManager.setLoggerFactory(mockFact);

    mockConfig = mock(IGlidepathAppConfig.class);

    when(mockConfig.getProperty("asd.intersections")).thenReturn("9945");

    GlidepathApplicationContext.getInstance().setAppConfigOverride(mockConfig);

    log = LogFactory.getLog(IntersectionProcessingTest.class);
    log.info("Setting up tests for IntersectionProcessingTest");
  }

  @After
  public void tearDown() throws Exception {
  }

  private class NodeData {

    public int choice;
    public double x;
    public double y;
    NodeData(int choice, double x, double y) {
      this.choice = choice;
      this.x = x;
      this.y = y;
    }
  }

  /**
   * Tests the interpolation of vehicle position between two nodes
   * @throws Exception
   */
  @Test
  public void testUpdateIntersections() throws Exception {
    MapData map = buildNewMap();
    SPAT spat = buildNewSpat();
    EadIntersectionManager intManager = new EadIntersectionManager();

    IntersectionData intData = new IntersectionData(map.getIntersections().get(0), spat.getIntersectionStateList().get(0));
    // Eastbound in-front of second intersection at TFHRC
    gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location vehicleLoc = new gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location(38.954974, -77.147745);
    
    MapMessage eadMap = TrafficSignalPlugin.convertMapMessage(intData);
    SpatMessage eadSpat = TrafficSignalPlugin.convertSpatMessage(intData);

    gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData eadIntData = new gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData();
    eadIntData.map = eadMap;
    eadIntData.spat = eadSpat;
    
    intManager.updateIntersections(Arrays.asList(eadIntData), vehicleLoc);
  }

  private SPAT buildNewSpat() {
    SPAT spat = messageFactory.newFromType(SPAT._TYPE);
    IntersectionState intersectionState = messageFactory.newFromType(IntersectionState._TYPE);

    intersectionState.getId().setId((short)9945);
    intersectionState.setMoy(375248); // MOY may not be needed for test
    intersectionState.setMoyExists(true);
    intersectionState.setTimeStamp((float)0.460000008345); // Timestamp may not be needed for test
    intersectionState.setTimeStampExists(true);

    MovementState movementState1 = messageFactory.newFromType(MovementState._TYPE);
    movementState1.setSignalGroup((byte)4);
    MovementEvent movementEvent1 = messageFactory.newFromType(MovementEvent._TYPE);
    movementEvent1.getEventState().setMovementPhaseState((byte)5);
    movementEvent1.getTiming().setMinEndTime((float)480.399993896);
    movementEvent1.getTiming().setMaxEndTime((float)520.900024414);
    movementEvent1.getTiming().setMaxEndTimeExists(true);
    movementEvent1.setTimingExists(true);

    movementState1.getMovementEventList().add(movementEvent1);

    MovementState movementState2 = messageFactory.newFromType(MovementState._TYPE);
    movementState2.setSignalGroup((byte)2);
    MovementEvent movementEvent2 = messageFactory.newFromType(MovementEvent._TYPE);
    movementEvent2.getEventState().setMovementPhaseState((byte)5);
    movementEvent2.getTiming().setMinEndTime((float)484.899993896);
    movementEvent2.getTiming().setMaxEndTime((float)525.400024414);
    movementEvent2.getTiming().setMaxEndTimeExists(true);
    movementEvent2.setTimingExists(true);

    movementState2.getMovementEventList().add(movementEvent2);

    intersectionState.getMovementList().add(movementState1);
    intersectionState.getMovementList().add(movementState2);
    
    spat.getIntersectionStateList().add(intersectionState);

    return spat;
  }

  private MapData buildNewMap() {
    MapData map = messageFactory.newFromType(MapData._TYPE);

    map.getHeader().setSeq(1);
    map.getHeader().setStamp(new Time(1537279681, 569000000));
    map.getHeader().setFrameId("0");

    map.setIntersectionsExists(true);

    IntersectionGeometry intersection = messageFactory.newFromType(IntersectionGeometry._TYPE);

    intersection.getId().setId((short)9945);
    intersection.getRefPoint().setElevationExists(true);
    intersection.getRefPoint().setElevation((float)39.0);
    intersection.getRefPoint().setLatitude(38.9550311);
    intersection.getRefPoint().setLongitude(-77.1473578);
    intersection.setLaneWidth((float)2.74000000954);
    intersection.setLaneWidthExists(true);


    // LANE 1 4 indent -
    GenericLane l1 = messageFactory.newFromType(GenericLane._TYPE);
    l1.setLaneId((byte)1);
    l1.setIngressApproach((byte)1);
    l1.setIngressApproachExists(true);
    l1.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)1);
    l1.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL1 = new ArrayList<>();

    nodeDataForL1.add(new NodeData(2, 3.47000002861, -11.0399999619));
    nodeDataForL1.add(new NodeData(0, -0.170000001788, -3.65000009537));
    nodeDataForL1.add(new NodeData(0, -0.460000008345, -2.8900001049));
    nodeDataForL1.add(new NodeData(0, -1.5, -3.65000009537));
    nodeDataForL1.add(new NodeData(0, -2.72000002861, -2.83999991417));
    nodeDataForL1.add(new NodeData(0, -4.80999994278, -2.8900001049));
    nodeDataForL1.add(new NodeData(1, -5.44999980927, -2.43000006676));
    nodeDataForL1.add(new NodeData(1, -6.07999992371, -1.73000001907));
    nodeDataForL1.add(new NodeData(1, -9.56000041962, -3.3599998951));
    nodeDataForL1.add(new NodeData(1, -9.56000041962, -3.3599998951));

    for (NodeData n : nodeDataForL1) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l1.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }

    j2735_msgs.Connection connectedLane1 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane1.getConnectingLane().setLane((byte)6);
    connectedLane1.setSignalGroup((byte)2);
    connectedLane1.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane2 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane2.getConnectingLane().setLane((byte)7);
    connectedLane2.setSignalGroup((byte)2);
    connectedLane2.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane3 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane3.getConnectingLane().setLane((byte)8);
    connectedLane3.setSignalGroup((byte)2);
    connectedLane3.setSignalGroupExists(true);

    l1.setConnectToList(Arrays.asList(connectedLane1,connectedLane2,connectedLane3));

    l1.setConnectsToExists(true);

    // LANE 5
    GenericLane l5 = messageFactory.newFromType(GenericLane._TYPE);
    l5.setLaneId((byte)1);
    l5.setIngressApproach((byte)0);
    l5.setIngressApproachExists(true);
    l5.setEgressApproach((byte)5);
    l5.setEgressApproachExists(true);
    l5.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)2);
    l5.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL5 = new ArrayList<>();

    nodeDataForL5.add(new NodeData(2, 0.340000003576, -11.3900003433));
    nodeDataForL5.add(new NodeData(0, -0.170000001788, -3.24000000954));
    nodeDataForL5.add(new NodeData(0, -1.78999996185, -4.80999994278));
    nodeDataForL5.add(new NodeData(0, -2.59999990463, -2.83999991417));
    nodeDataForL5.add(new NodeData(0, -4.0, -2.30999994278));
    nodeDataForL5.add(new NodeData(1, -5.55999994278, -1.85000002384));
    nodeDataForL5.add(new NodeData(1, -9.78999996185, -2.8900001049));
    nodeDataForL5.add(new NodeData(1, -6.36999988556, -1.85000002384));

    for (NodeData n : nodeDataForL5) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l5.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }

    // LANE 6
    GenericLane l6 = messageFactory.newFromType(GenericLane._TYPE);
    l6.setLaneId((byte)6);
    l6.setEgressApproach((byte)6);
    l6.setEgressApproachExists(true);
    l6.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)2);
    l6.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL6 = new ArrayList<>();

    nodeDataForL6.add(new NodeData(2, 16.3999996185, 2.51999998093));
    nodeDataForL6.add(new NodeData(1, 8.46000003815, 2.95000004768));
    nodeDataForL6.add(new NodeData(1, 9.56000041962, 4.05000019073));
    nodeDataForL6.add(new NodeData(0, 5.03999996185, 3.29999995232));
    nodeDataForL6.add(new NodeData(0, 3.71000003815, 3.71000003815));
    nodeDataForL6.add(new NodeData(0, 3.06999993324, 3.18000006676));
    nodeDataForL6.add(new NodeData(1, 3.42000007629, 5.67999982834));
    nodeDataForL6.add(new NodeData(1, 2.01999998093, 6.78000020981));

    for (NodeData n : nodeDataForL6) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l6.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }

    // LANE 2
    GenericLane l2 = messageFactory.newFromType(GenericLane._TYPE);
    l2.setLaneId((byte)2);
    l2.setIngressApproach((byte)2);
    l2.setIngressApproachExists(true);
    l2.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)1);
    l2.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL2 = new ArrayList<>();

    nodeDataForL2.add(new NodeData(2, 15.529999733, 5.76000022888));
    nodeDataForL2.add(new NodeData(1, 6.88999986649, 2.1400001049));
    nodeDataForL2.add(new NodeData(1, 7.30000019073, 3.00999999046));
    nodeDataForL2.add(new NodeData(1, 6.19999980927, 3.52999997139));
    nodeDataForL2.add(new NodeData(0, 4.51999998093, 3.13000011444));
    nodeDataForL2.add(new NodeData(0, 3.13000011444, 3.75999999046));
    nodeDataForL2.add(new NodeData(1, 2.59999990463, 6.59999990463));

    for (NodeData n : nodeDataForL6) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l6.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }
   
    j2735_msgs.Connection connectedLane1ForL2 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane1ForL2.getConnectingLane().setLane((byte)5);
    connectedLane1ForL2.setSignalGroup((byte)4);
    connectedLane1ForL2.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane2ForL2 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane2ForL2.getConnectingLane().setLane((byte)7);
    connectedLane2ForL2.setSignalGroup((byte)4);
    connectedLane2ForL2.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane3ForL2 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane3ForL2.getConnectingLane().setLane((byte)8);
    connectedLane3ForL2.setSignalGroup((byte)4);
    connectedLane3ForL2.setSignalGroupExists(true);

    l2.setConnectToList(Arrays.asList(connectedLane1ForL2,connectedLane2ForL2,connectedLane3ForL2));
    l2.setConnectsToExists(true);

    // LANE 3
    GenericLane l3 = messageFactory.newFromType(GenericLane._TYPE);
    l3.setLaneId((byte)3);
    l3.setIngressApproach((byte)3);
    l3.setIngressApproachExists(true);
    l3.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)1);
    l3.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL3 = new ArrayList<>();

    nodeDataForL2.add(new NodeData(2, -4.51999998093, 12.8999996185));
    nodeDataForL2.add(new NodeData(1, -0.75, 5.21000003815));
    nodeDataForL2.add(new NodeData(1, -1.55999994278, 5.61999988556));
    nodeDataForL2.add(new NodeData(0, -1.33000004292, 3.52999997139));
    nodeDataForL2.add(new NodeData(0, -2.95000004768, 4.57999992371));
    nodeDataForL2.add(new NodeData(0, -2.83999991417, 2.95000004768));
    nodeDataForL2.add(new NodeData(0, -3.81999993324, 2.8900001049));
    nodeDataForL2.add(new NodeData(0, -4.23000001907, 2.1400001049));
    nodeDataForL2.add(new NodeData(0, -3.88000011444, 1.78999996185));

    for (NodeData n : nodeDataForL3) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l3.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }
   
    j2735_msgs.Connection connectedLane1ForL3 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane1ForL3.getConnectingLane().setLane((byte)5);
    connectedLane1ForL3.setSignalGroup((byte)2);
    connectedLane1ForL3.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane2ForL3 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane2ForL3.getConnectingLane().setLane((byte)6);
    connectedLane2ForL3.setSignalGroup((byte)2);
    connectedLane2ForL3.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane3ForL3 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane3ForL3.getConnectingLane().setLane((byte)8);
    connectedLane3ForL3.setSignalGroup((byte)2);
    connectedLane3ForL3.setSignalGroupExists(true);

    l3.setConnectToList(Arrays.asList(connectedLane1ForL3,connectedLane2ForL3,connectedLane3ForL3));
    l3.setConnectsToExists(true);

    // LANE 7
    GenericLane l7 = messageFactory.newFromType(GenericLane._TYPE);
    l7.setLaneId((byte)7);
    l7.setEgressApproach((byte)7);
    l7.setEgressApproachExists(true);
    l7.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)2);
    l7.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL7 = new ArrayList<>();

    nodeDataForL7.add(new NodeData(2, -1.62000000477, 13.7700004578));
    nodeDataForL7.add(new NodeData(0, -0.75, 4.80999994278));
    nodeDataForL7.add(new NodeData(1, -1.44000005722, 5.61999988556));
    nodeDataForL7.add(new NodeData(1, -2.77999997139, 6.65999984741));
    nodeDataForL7.add(new NodeData(0, -2.66000008583, 3.65000009537));
    nodeDataForL7.add(new NodeData(0, -3.18000006676, 3.42000007629));
    nodeDataForL7.add(new NodeData(0, -2.54999995232, 1.97000002861));
    nodeDataForL7.add(new NodeData(0, -3.94000005722, 2.07999992371));

    for (NodeData n : nodeDataForL7) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l7.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }

    // LANE 8
    GenericLane l8 = messageFactory.newFromType(GenericLane._TYPE);
    l8.setLaneId((byte)8);
    l8.setEgressApproach((byte)8);
    l8.setEgressApproachExists(true);
    l8.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)2);
    l8.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL8 = new ArrayList<>();

    nodeDataForL8.add(new NodeData(2, -18.6100006104, -1.17999994755));
    nodeDataForL8.add(new NodeData(1, -10.1400003433, -1.97000002861));
    nodeDataForL8.add(new NodeData(2, -12.0600004196, -2.36999988556));
    nodeDataForL8.add(new NodeData(2, -11.4200000763, -2.25999999046));
    nodeDataForL8.add(new NodeData(1, -9.97000026703, -1.67999994755));
    nodeDataForL8.add(new NodeData(1, -6.13999986649, -0.860000014305));
    nodeDataForL8.add(new NodeData(1, -6.32000017166, -0.629999995232));
    nodeDataForL8.add(new NodeData(0, 0.0500000007451, 0.0));
    nodeDataForL8.add(new NodeData(0, -4.28999996185, -0.170000001788));

    for (NodeData n : nodeDataForL8) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l8.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }
    
    // LANE 4
    GenericLane l4 = messageFactory.newFromType(GenericLane._TYPE);
    l4.setLaneId((byte)4);
    l4.setIngressApproach((byte)4);
    l4.setIngressApproachExists(true);
    l4.getLaneAttributes().getDirectionalUse().setLaneDirection((byte)1);
    l4.getNodeList().setChoice((byte)0);
    

    List<NodeData> nodeDataForL4 = new ArrayList<>();

    nodeDataForL4.add(new NodeData(2, -17.9099998474, -4.01999998093));
    nodeDataForL4.add(new NodeData(2, -11.1899995804, -2.49000000954));
    nodeDataForL4.add(new NodeData(2, -10.779999733, -2.59999990463));
    nodeDataForL4.add(new NodeData(1, -9.44999980927, -1.62000000477));
    nodeDataForL4.add(new NodeData(1, -10.1999998093, -2.07999992371));
    nodeDataForL4.add(new NodeData(1, -5.90999984741, -0.920000016689));
    nodeDataForL4.add(new NodeData(0, -3.75999999046, -0.40000000596));
    nodeDataForL4.add(new NodeData(1, -6.65999984741, -0.460000008345));

    for (NodeData n : nodeDataForL4) {
      NodeXY nodeXY = messageFactory.newFromType(NodeXY._TYPE);
      nodeXY.getDelta().setChoice((byte)n.choice);
      nodeXY.getDelta().setX((float)n.x);
      nodeXY.getDelta().setY((float)n.y);
      l4.getNodeList().getNodes().getNodeSetXy().add(nodeXY);
    }
   
    j2735_msgs.Connection connectedLane1ForL4 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane1ForL4.getConnectingLane().setLane((byte)5);
    connectedLane1ForL4.setSignalGroup((byte)4);
    connectedLane1ForL4.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane2ForL4 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane2ForL4.getConnectingLane().setLane((byte)6);
    connectedLane2ForL4.setSignalGroup((byte)4);
    connectedLane2ForL4.setSignalGroupExists(true);

    j2735_msgs.Connection connectedLane3ForL4 = messageFactory.newFromType(j2735_msgs.Connection._TYPE);
    connectedLane3ForL4.getConnectingLane().setLane((byte)7);
    connectedLane3ForL4.setSignalGroup((byte)4);
    connectedLane3ForL4.setSignalGroupExists(true);

    l4.setConnectToList(Arrays.asList(connectedLane1ForL4,connectedLane2ForL4,connectedLane3ForL4));
    l4.setConnectsToExists(true);

    // Add lanes to intersection
    intersection.setLaneList(Arrays.asList(l1,l2,l3,l4,l5,l6,l7,l8));
    
    // Add intersection to map
    map.getIntersections().add(intersection);
    
    return map;
  }
}
