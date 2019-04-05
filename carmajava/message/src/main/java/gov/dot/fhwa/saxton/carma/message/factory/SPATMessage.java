package gov.dot.fhwa.saxton.carma.message.factory;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import cav_msgs.ByteArray;
import j2735_msgs.IntersectionState;
import j2735_msgs.MovementEvent;
import j2735_msgs.MovementState;
import j2735_msgs.SPAT;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class SPATMessage implements IMessage<SPAT> {
    
    protected static final int INTERSECTION_DATA_SIZE = 8;
    protected static final int MAX_STATE_LIST_SIZE = 255;
    protected static final int MAX_EVENT_LIST_SIZE = 16;
    protected static final int EVENT_DATA_SIZE = 9;
    // the extra space is for state region ID
    protected static final int STATE_DATA_SIZE = EVENT_DATA_SIZE * MAX_EVENT_LIST_SIZE + 1;  
    
    protected SaxtonLogger log;
    protected MessageFactory messageFactory;
    
    public SPATMessage(MessageFactory factory, SaxtonLogger logger) {
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
     * This is the declaration for native method. It will take encoded SPAT byte array as input.
     * It will decode the message and set other byte array inputs values.
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeSPAT(byte[] encodedArray, int[] intersectionData, int[][] movementStatesData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This parser currently does not support encode SPAT messages 
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
        int[][] statesData = new int[MAX_STATE_LIST_SIZE][STATE_DATA_SIZE];
        SPAT spat = messageFactory.newFromType(SPAT._TYPE);
        int res = decodeSPAT(encodedMsg, intersectionData, statesData);
        if (res == -1) {
            log.warn("SPATMessage cannot be decoded.");
            return new MessageContainer("SPAT", null);
        }
        // Copy data from arrays into the ROS message
        // IntersectionData array: {SPATTimestamp, SPATtimestampExist, intersectionID, revision, moy, moyExist, intersectionTimestamp, intersectionTimestampExist}
        // TODO Current version only support one intersection in each SPAT message
        spat.setTimeStampExists(intersectionData[1] == 1);
        if(spat.getTimeStampExists()) {
            spat.setTimeStamp(intersectionData[0]);
        }
        IntersectionState intersection = messageFactory.newFromType(IntersectionState._TYPE);
        intersection.getId().setId((short) intersectionData[2]);
        intersection.setRevision((byte) intersectionData[3]);
        intersection.setMoyExists(intersectionData[5] == 1);
        if(intersection.getMoyExists()) {
            intersection.setMoy(intersectionData[4]);
        }
        intersection.setTimeStampExists(intersectionData[7] == 1);
        if(intersection.getTimeStampExists()) {
            intersection.setTimeStamp(intersectionData[6]);
        }
        for(int i = 0; i < MAX_STATE_LIST_SIZE; i++) {
            // row vector of statesData is: {signalGroupId, movementEvent * 16}
            // each movementEvent vector is: {movementPhaseState, startTime, startTimeExist, minEndTime, maxEndTime, maxEndTimeExist
            //                                nextTime, nextTimeExist, timingChangeDetailsExist}
            // -1 signal group ID indicates there is no more movement state elements
            if(statesData[i][0] == -1) {
                break;
            }
            MovementState state = messageFactory.newFromType(MovementState._TYPE);
            state.setSignalGroup((byte) statesData[i][0]);
            for(int j = 0; j < MAX_EVENT_LIST_SIZE; j ++) {
                // -1 movementPhaseState indicates there is no more events
                if(statesData[i][j * EVENT_DATA_SIZE + 1] == -1) {
                    break;
                }
                MovementEvent event = messageFactory.newFromType(MovementEvent._TYPE);
                event.getEventState().setMovementPhaseState((byte) statesData[i][j * EVENT_DATA_SIZE + 1]);
                event.setTimingExists(statesData[i][j * EVENT_DATA_SIZE + 9] == 1);
                if(event.getTimingExists()) {
                    event.getTiming().setStartTimeExists(statesData[i][j * EVENT_DATA_SIZE + 3] == 1);
                    if(event.getTiming().getStartTimeExists()) {
                        event.getTiming().setStartTime((short) statesData[i][j * EVENT_DATA_SIZE + 2]);
                    }
                    event.getTiming().setMinEndTime((short) statesData[i][j * EVENT_DATA_SIZE + 4]);
                    event.getTiming().setMaxEndTimeExists(statesData[i][j * EVENT_DATA_SIZE + 6] == 1);
                    if(event.getTiming().getMaxEndTimeExists()) {
                        event.getTiming().setMaxEndTime((short) statesData[i][j * EVENT_DATA_SIZE + 5]);
                    }
                    event.getTiming().setNextTimeExists(statesData[i][j * EVENT_DATA_SIZE + 8] == 1);
                    if(event.getTiming().getNextTimeExists()) {
                        event.getTiming().setNextTime((short) statesData[i][j * EVENT_DATA_SIZE + 7]);
                    }
                }
                state.getStateTimeSpeed().getMovementEventList().add(event);
            }
            intersection.getStates().getMovementList().add(state);
        }
        spat.getIntersections().getIntersectionStateList().add(intersection);
        return new MessageContainer("SPAT", spat);
    }

}
