package gov.dot.fhwa.saxton.carma.message.factory;

import java.util.LinkedList;
import java.util.List;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import cav_msgs.ByteArray;
import cav_msgs.GenericLane;
import cav_msgs.IntersectionGeometry;
import cav_msgs.IntersectionReferenceID;
import cav_msgs.LaneAttributes;
import cav_msgs.LaneList;
import cav_msgs.MapData;
import cav_msgs.Position3D;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MapMessage implements IMessage<MapData>{

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
            e.printStackTrace();
        }
    }
    
    /**
     * This is the declaration for native method. It will take encoded MAP byte array as input.
     * It will decode the message and set other byte array inputs values. 
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMap(byte[] encodedArray, Object map_object, int[] intersectionData, int[] laneIDData,
            int[] ingressApproachData, int[] egressApproachData, int[] laneDirectionData, int[] laneTypeData, int[][] nodeOffsetData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This class currently does not support encode MAP message 
        throw new UnsupportedOperationException();
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encodedMsg = new byte[buffer.capacity()];
        for (int i = 0; i < buffer.capacity(); i++) {
            encodedMsg[i] = buffer.getByte(i);
        }
        int[] intersectionData = new int[9];
        int[] laneIDData = new int[255];
        int[] ingressApproachData = new int[255];
        int[] egressApproachData = new int[255];
        int[] laneDirectionData = new int[255];
        int[] laneTypeData = new int[255];
        int[][] nodeOffsetData = new int[255][189];
        MapData map = messageFactory.newFromType(MapData._TYPE);
        int res = decodeMap(encodedMsg, map, intersectionData, laneIDData, ingressApproachData,
                                    egressApproachData, laneDirectionData, laneTypeData, nodeOffsetData);
        if (res == -1) {
            log.warn("MapMessage cannot be decoded.");
            return new MessageContainer("MAP", null);
        }
        map.setIntersectionsExists(intersectionData[8] == 1);
        if(map.getIntersectionsExists()) {
            IntersectionGeometry intersection = messageFactory.newFromType(IntersectionGeometry._TYPE);
            IntersectionReferenceID id = messageFactory.newFromType(IntersectionReferenceID._TYPE);
            id.setId((short) intersectionData[0]);
            intersection.setId(id);
            intersection.setRevision((byte) intersectionData[1]);
            Position3D refPoint = messageFactory.newFromType(Position3D._TYPE);
            refPoint.setLatitude(intersectionData[2]);
            refPoint.setLongitude(intersectionData[3]);
            refPoint.setElevationExists(intersectionData[5] == 1);
            if(refPoint.getElevationExists()) {
                refPoint.setElevation(intersectionData[4]);
            }
            intersection.setRefPoint(refPoint);
            intersection.setLaneWidthExists(intersectionData[7] == 1);
            if(intersection.getLaneWidthExists()) {
                intersection.setLaneWidth((short) intersectionData[6]);
            }
            LaneList laneList = messageFactory.newFromType(LaneList._TYPE);
            List<GenericLane> lanes = new LinkedList<>();
            for(int i = 0; i < laneIDData.length; i++) {
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
                LaneAttributes attributes = messageFactory.newFromType(LaneAttributes._TYPE);
                attributes.getDirectionalUse().setLaneDirection((byte) 0x11); // Lets see if it works
                lane.setLaneAttributes(attributes);
                lanes.add(lane);
            }
            laneList.setLaneList(lanes);
            intersection.setLaneSet(laneList);
            map.getIntersections().add(intersection);
        }
        return new MessageContainer("MAP", map);
    }

}
