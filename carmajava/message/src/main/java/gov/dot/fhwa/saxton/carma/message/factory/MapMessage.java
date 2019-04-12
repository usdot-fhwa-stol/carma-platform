package gov.dot.fhwa.saxton.carma.message.factory;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import cav_msgs.ByteArray;
import j2735_msgs.Connection;
import j2735_msgs.GenericLane;
import j2735_msgs.IntersectionGeometry;
import j2735_msgs.MapData;
import j2735_msgs.NodeListXY;
import j2735_msgs.NodeOffsetPointXY;
import j2735_msgs.NodeXY;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MapMessage implements IMessage<MapData>{
    
    protected static final int MAX_NODE_LIST_SIZE = 63;
    protected static final int INTERSECTION_DATA_SIZE = 9;
    protected static final int MAX_LANE_LIST_SIZE = 255;
    protected static final int NODE_OFFSETS_DATA_SIZE = 189;
    protected static final int CONNECTION_DATA_SIZE = 32;

    protected SaxtonLogger log;
    protected MessageFactory messageFactory;
    
    public MapMessage(MessageFactory factory, SaxtonLogger logger) {
        this.log            = logger;
        this.messageFactory = factory;
    }

    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
        }
    }
    
    /**
     * This is the declaration for native method. It will take encoded MAP byte array as input.
     * It will decode the message and set other byte array inputs values. 
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMap(byte[] encodedArray, Object map_object, int[] intersectionData, int[] laneIDData,
            int[] ingressApproachData, int[] egressApproachData, int[] laneDirectionData, int[] laneTypeData, int[][] nodeOffsetData, int[][] connectsToData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This parser currently does not support encode MAP messages 
        throw new UnsupportedOperationException();
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        // Copy binary message from ChannelBuffer to byte array
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encodedMsg = new byte[buffer.capacity()];
        for (int i = 0; i < buffer.capacity(); i++) {
            encodedMsg[i] = buffer.getByte(i);
        }
        // Initialize empty arrays to hold MAP data
        int[] intersectionData = new int[INTERSECTION_DATA_SIZE];
        int[] laneIDData = new int[MAX_LANE_LIST_SIZE];
        int[] ingressApproachData = new int[MAX_LANE_LIST_SIZE];
        int[] egressApproachData = new int[MAX_LANE_LIST_SIZE];
        int[] laneDirectionData = new int[MAX_LANE_LIST_SIZE];
        int[] laneTypeData = new int[MAX_LANE_LIST_SIZE];
        int[][] nodeOffsetData = new int[MAX_LANE_LIST_SIZE][NODE_OFFSETS_DATA_SIZE];
        int[][] connectsToData = new int[MAX_LANE_LIST_SIZE][CONNECTION_DATA_SIZE];
        MapData map = messageFactory.newFromType(MapData._TYPE);
        int res = decodeMap(encodedMsg, map, intersectionData, laneIDData, ingressApproachData,
                            egressApproachData, laneDirectionData, laneTypeData, nodeOffsetData, connectsToData);
        if (res == -1) {
            log.warn("MapMessage cannot be decoded.");
            return new MessageContainer("MAP", null);
        }
        // Copy data from arrays into the ROS MAP message
        // IntersectionData array: {intersectionId, revision, lat, long, elevation, elevationExist, laneWidth, laneWidthExist, intersectionExist}
        // TODO Current version only support one intersection in each MAP message 
        map.setIntersectionsExists(intersectionData[8] == 1);
        if(map.getIntersectionsExists()) {
            IntersectionGeometry intersection = messageFactory.newFromType(IntersectionGeometry._TYPE);
            intersection.getId().setId((short) intersectionData[0]);
            intersection.setRevision((byte) intersectionData[1]);
            intersection.getRefPoint().setLatitude(intersectionData[2]);
            intersection.getRefPoint().setLongitude(intersectionData[3]);
            intersection.getRefPoint().setElevationExists(intersectionData[5] == 1);
            if(intersection.getRefPoint().getElevationExists()) {
                intersection.getRefPoint().setElevation(intersectionData[4]);
            }
            intersection.setLaneWidthExists(intersectionData[7] == 1);
            if(intersection.getLaneWidthExists()) {
                intersection.setLaneWidth((short) intersectionData[6]);
            }
            // Copy lane data to each JAVA lane object
            for(int i = 0; i < laneIDData.length; i++) {
                // -1 lane ID indicates no more lanes in this intersection
                if(laneIDData[i] == -1) {
                    break;
                }
                GenericLane lane = messageFactory.newFromType(GenericLane._TYPE);
                lane.setLaneId((byte) laneIDData[i]);
                lane.setIngressApproachExists(ingressApproachData[i] != -1);
                if(lane.getIngressApproachExists()) {
                    lane.setIngressApproach((byte) ingressApproachData[i]);
                }
                lane.setEgressApproachExists(egressApproachData[i] != -1);
                if(lane.getEgressApproachExists()) {
                    lane.setEgressApproach((byte) egressApproachData[i]);
                }
                // 0b11 means {ingressPath, egressPath}, 0b10 means {ingressPath}, 0b01 means {egressPath} and 0b00 means an empty set
                // In asn1c library, it uses the first two bits of a byte to represent a bit string of length 2
                lane.getLaneAttributes().getDirectionalUse().setLaneDirection((byte) (laneDirectionData[i] >> 6));
                // In asn1c library, the lane type enum starts from 1 
                lane.getLaneAttributes().getLaneType().setChoice((byte) (laneTypeData[i] - 1));
                // Current version only supports the choice of NODE_SET_XY
                lane.getNodeList().setChoice(NodeListXY.NODE_SET_XY);
                // Add all valid node offsets into the corresponding lane
                for(int j = 0; j < MAX_NODE_LIST_SIZE; j++) {
                    int nodeType = nodeOffsetData[i][j * 3];
                    // 0 node type indicates no more nodes in this lane
                    if(nodeType == 0) {
                        break;
                    }
                    NodeXY node = messageFactory.newFromType(NodeXY._TYPE);
                    // set node offset point choice value based on its node type
                    switch(nodeType) {
                    case 1:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY1);
                        node.getDelta().getNodeXy1().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy1().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 2:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY2);
                        node.getDelta().getNodeXy2().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy2().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 3:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY3);
                        node.getDelta().getNodeXy3().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy3().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 4:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY4);
                        node.getDelta().getNodeXy4().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy4().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 5:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY5);
                        node.getDelta().getNodeXy5().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy5().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 6:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY6);
                        node.getDelta().getNodeXy6().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy6().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 7:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_LATLON);
                        node.getDelta().getNodeLatlon().setLatitude(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeLatlon().setLongitude(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    default:
                        break;
                    }
                    lane.getNodeList().getNodes().getNodeSetXy().add(node);
                }
                for(int j = 0; j < CONNECTION_DATA_SIZE; j += 2) {
                    if(connectsToData[i][j] == -1) {
                        break;
                    } else {
                        Connection connection = messageFactory.newFromType(Connection._TYPE);
                        connection.getConnectingLane().setLane((short) connectsToData[i][j]);
                        //connection.getConnectingLane().setManeuverExists(); // TODO This field is not populated
                        //connection.getConnectingLane().getManeuver().setAllowedManeuvers(); // TODO This field is not populated. It should also be filled at the generic lane level
                        connection.setSignalGroupExists(connectsToData[i][j + 1] != -1);
                        if(connection.getSignalGroupExists()) {
                            connection.setSignalGroup((byte) connectsToData[i][j + 1]);
                        }
                        lane.setConnectsToExists(true);
                        lane.getConnectsTo().getConnectToList().add(connection);
                    }
                }
                intersection.getLaneSet().getLaneList().add(lane);
            }
            map.getIntersections().add(intersection);
        }
        map.getHeader().setFrameId("0");
        return new MessageContainer("MAP", map);
    }

}
