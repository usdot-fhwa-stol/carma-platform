package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.PhaseDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.UnpackUtils;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import javassist.bytecode.ByteArray;
import org.joda.time.DateTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * SPAT Message
 *
 * User: ferenced
 * Date: 1/16/15
 * Time: 3:19 PM
 *
 */
public class SpatMessage implements ISpatMessage {

    private static ILogger logger = LoggerManager.getLogger(SpatMessage.class);

    private byte ID = (byte) 0x8d;
    private int version;
    private int payloadLength;

    private int intersectionId;
    private int status;
    private DateTime timestamp;

    private List<Movement> movements = new ArrayList<Movement>();
    private Movement movement = null;
    private static Movement lastMovement = null;

    // objects decoded in a SPAT message
    public static final int SO_INTERSECTION_ID = 0x01;
    public static final int SO_INTERSECTION_STATUS = 0x02;
    public static final int SO_MESSAGE_TIMESTAMP = 0x03;
    public static final int SO_MOVEMENT = 0x04;
    public static final int SO_LANE_SET = 0x05;
    public static final int SO_CURRENT_STATE = 0x06;
    public static final int SO_MIN_TIME_REMAINING = 0x07;
    public static final int SO_MAX_TIME_REMAINING = 0x08;
    public static final int SO_YELLOW_STATE = 0x09;
    public static final int SO_YELLOW_TIME = 0x0A;
    public static final int SO_PEDESTRIAN_DETECTED = 0x0B;
    public static final int SO_VEHICLE_PEDESTRIAN_COUNT = 0x0C;
    public static final int SO_EOB = 0xFF;

    public static final int SO_INDEX_INTERSECTION_ID = 4;

    public SpatMessage()   {
    }

    public boolean parse(byte[] data)   {

        boolean result = true;

        try   {
            validate(data);

            int nextIndex = SO_INDEX_INTERSECTION_ID;

            while (nextIndex > 0)   {
                nextIndex = processObject(data, nextIndex);
            }
        }
        catch(Exception e)   {
            logger.error("SPAT", "Error parsing SPAT message: " + e.getMessage());
            result = false;
        }

        return result;
    }

    /**
     *
     * @param objectBuf
     * @param startIndex
     * @return
     * @throws Exception
     */
    private int processObject(byte[] objectBuf, int startIndex) throws Exception   {
        byte objectId = (byte) objectBuf[startIndex];

        if (objectId == (byte) SO_EOB)   {
           return -1;
        }

        int objectLength = objectBuf[startIndex + 1];

        switch (objectId)   {
            case SO_INTERSECTION_ID:
                intersectionId = ByteArray.read32bit(objectBuf, startIndex + 2);
                break;

            case SO_INTERSECTION_STATUS:
                status = objectBuf[startIndex + 2] & 0x0F;
                break;

            case SO_MESSAGE_TIMESTAMP:
                byte[] baSeconds = Arrays.copyOfRange(objectBuf, startIndex + 2, startIndex + 2 + 4);

                long seconds = UnpackUtils.getInstance().unpackU32BigEndian(baSeconds);
                int tenths = objectBuf[startIndex + 6];

                timestamp = new DateTime(seconds * 1000 + tenths * 100);
                break;

            case SO_MOVEMENT:
                // create new movement
                movement = new Movement();
                objectLength = - 1;       // no payload with movement object
                break;

            case SO_LANE_SET:
                byte[] lanesetPayload = Arrays.copyOfRange(objectBuf, startIndex + 2, startIndex + 2 + objectLength);

                for (int i=0; i<lanesetPayload.length; i+=2)   {
                    LaneSet laneSet = new LaneSet(lanesetPayload[i + 1], lanesetPayload[i]);
                    movement.addLaneSet(laneSet);
                }

                break;

            case SO_CURRENT_STATE:
                byte[] stateArray = Arrays.copyOfRange(objectBuf, startIndex + 2, startIndex + 2 + objectLength);
                int state = 0;

                if (stateArray.length == 1)   {
                    state = stateArray[0];
                }
                else if (stateArray.length == 2)   {
                    state = ByteArray.readU16bit(stateArray, 0);
                }
                else if (stateArray.length == 4)   {
                    state = ByteArray.read32bit(stateArray, 0);
                }

                movement.setCurrentState(state);
                break;

            case SO_MIN_TIME_REMAINING:
                int timeRemaining = ByteArray.readU16bit(objectBuf, startIndex + 2);
                movement.setMinTimeRemaining(timeRemaining);
                break;

            case SO_MAX_TIME_REMAINING:
                int maxTimeRemaining = ByteArray.readU16bit(objectBuf, startIndex + 2);
                movement.setMaxTimeRemaining(maxTimeRemaining);
                movements.add(movement);
                break;

            case SO_YELLOW_STATE:
            case SO_YELLOW_TIME:
            case SO_PEDESTRIAN_DETECTED:
            case SO_VEHICLE_PEDESTRIAN_COUNT:
            // these objects are optional, they weren't included when testing in the 'flashing' state, but the two
            // yellow objects were occasionally included when SPAT was switched to normal
                break;

            default:
                throw new Exception("Invalid Object ID in SPAT message.");

        }

        return startIndex + 2 + objectLength;
    }

    /**
     * Validates that this byte array is a SPAT message
     *
     * Ensures valid SPAT ID (0x8D)
     * Ensures last byte == 0xFF based on payload length
     *
     * @param data
     * @throws Exception
     */
    private void validate(byte[] data) throws Exception   {

        if (data[0] != ID)   {
            throw new Exception("Invalid ID (not 0x8D) for SPAT message.");
        }

        version = data[1] & 0xFF;

        payloadLength = ByteArray.readU16bit(data, 2);

        byte eob = data[2 + payloadLength + 1];

        if (eob != (byte) SO_EOB)   {
            throw new Exception("Invalid EOB (not 0xFF) for SPAT message.");
        }

    }

    /**
     * Gets the spat data for the provided lane, assumes STRAIGHT lane
     *
     * The SpatConsumer returns a SPAT message in the data holder, so we need to wait until the MAP consumer has
     * provided a Lane ID.  The Executor will call this method to acquire the appropriate SPAT for that lane.
     *
     * NOTE: the FHWA 2009 format message only gives timing info on the current phase, which is all we can return
     * here. This is an additional limitation that wasn't present in Glidepath v1 because that version used
     * hard-coded timing data for all phases, allowing future prediction beyond what is broadcast by the signal.
     *
     * @param lane
     * @return DataElementHolder containing SignalPhase and timing information
     */
    public DataElementHolder getSpatForLane(int lane)   {
        DataElementHolder holder = new DataElementHolder();

        for (Movement movement : movements)   {
            for (LaneSet laneSet : movement.getLaneSets())   {
                if (laneSet.getLane() == lane && laneSet.isStraight())   {
                    // let's only log when movement changes for now
                    if (lastMovement != null)   {
                        if (lastMovement.getCurrentStateAsEnum() != movement.getCurrentStateAsEnum())   {
                            logger.debug("SPAT", " ########## THIS IS THE LANE #############");
                            logger.debug("SPAT", "LaneSet: " + laneSet.getLane() + "  :  " + laneSet.getMovementAsString());
                            logger.debug("SPAT", "State: " + movement.getCurrentStateAsEnum() + " : " + movement.getMinTimeRemaining() + " : " + movement.getMaxTimeRemaining());
                            logger.debug("SPAT", "Current State: " + movement.getCurrentState());
                        }
                    }

                    // In our timed signal, the signal actually uses MAX time to indicate when the signal will change
                    // we will use the configured signal phase times to correctly set the third phase
                    holder.put(DataElementKey.SIGNAL_PHASE, new PhaseDataElement(movement.getCurrentStateAsEnum()));
                    holder.put(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE, new DoubleDataElement(movement.getMaxTimeRemaining()));
                    lastMovement = movement;
                }
            }
        }

        return holder;
    }


    /**
     * Acquire actual min and max time in the raw spat for a particular lane.
     * This puts min time in next phase and max time in third phase.
     * NOT USED IN APP, ONLY USED BY OUR SPAT UTIL
     *
     * @param lane
     * @return DataElementHolder
     */
    public DataElementHolder getRawSpatForLane(int lane)   {
        DataElementHolder holder = new DataElementHolder();

        for (Movement movement : movements)   {
            for (LaneSet laneSet : movement.getLaneSets())   {
                if (laneSet.getLane() == lane && laneSet.isStraight())   {
                    holder.put(DataElementKey.SIGNAL_PHASE, new PhaseDataElement(movement.getCurrentStateAsEnum()));
                    // use min and max
                    holder.put(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE, new DoubleDataElement(movement.getMinTimeRemaining()));
                    holder.put(DataElementKey.SIGNAL_TIME_TO_THIRD_PHASE, new DoubleDataElement(movement.getMaxTimeRemaining()));
                    break;
                }
            }
        }

        return holder;
    }


    @Override
    public int getIntersectionId() {
        return intersectionId;
    }


    @Override
    public int getContentVersion() {
        return version;
    }

    // CARMA setters needed for type conversion
    public void setContentVersion(int version) {
        this.version = version;
    }

    public void setIntersectionId(int id) {
        this.intersectionId = id;
    }

    public void setMovements(List<Movement> movements) {
        this.movements = movements;
    }

    public void setTimeStamp(DateTime timeStamp) {
        this.timestamp = timeStamp;
    }

    public void setStatus(int status) {
        this.status = status;
    }


    public void dumpSpatMessage()    {

        logger.debug("SPAT", " ###### SPAT MESSAGE #####");
        logger.debug("SPAT", "Intersection ID: \t" + intersectionId);
        logger.debug("SPAT", "Version: \t" + version);
        logger.debug("SPAT", "Intersection Status: \t" + status);
        logger.debug("SPAT", "Timestamp: " + timestamp);

        for (Movement movement : movements)   {
            logger.debug("SPAT", "  ***** movement ******");
            for (LaneSet laneSet : movement.getLaneSets())   {
                logger.debug("SPAT", "LaneSet: " + laneSet.getLane() + "  :  " + laneSet.getMovementAsString());
            }

            logger.debug("SPAT", "Movement State: \t" + movement.getCurrentState());
            logger.debug("SPAT", "Min Time: \t" + movement.getMinTimeRemaining());
            logger.debug("SPAT", "Max Time: \t" + movement.getMaxTimeRemaining());
        }
    }

    public String getSpatMessageAsString()    {

        StringBuffer sb = new StringBuffer();
        sb.append(" ###### SPAT MESSAGE #####");

        sb.append(" ###### SPAT MESSAGE #####\n");
        sb.append("Intersection ID: \t" + intersectionId + "\n");
        sb.append("Version: \t" + version + "\n");
        sb.append("Intersection Status: \t" + status + "\n");
        sb.append("Timestamp: " + timestamp + "\n");

        for (Movement movement : movements)   {
            sb.append("  ***** movement ******" + "\n");
            for (LaneSet laneSet : movement.getLaneSets())   {
                sb.append("LaneSet: " + laneSet.getLane() + "  :  " + laneSet.getMovementAsString() + "\n");
            }

            sb.append("Movement State: \t" + movement.getCurrentState() + "\n");
            sb.append("Min Time: \t" + movement.getMinTimeRemaining() + "\n");
            sb.append("Max Time: \t" + movement.getMaxTimeRemaining() + "\n");
        }

        return sb.toString();

    }

}
