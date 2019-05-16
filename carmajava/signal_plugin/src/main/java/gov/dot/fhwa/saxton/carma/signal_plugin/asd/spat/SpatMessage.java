/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.PhaseDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import org.joda.time.DateTime;

import java.util.ArrayList;
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

    private static final ILogger logger = LoggerManager.getLogger(SpatMessage.class);

    private int version;

    private int intersectionId;
    private int status;
    private DateTime timestamp;

    private List<Movement> movements = new ArrayList<Movement>();
    
    public SpatMessage()   {
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
                    // In our timed signal, the signal actually uses MAX time to indicate when the signal will change
                    // we will use the configured signal phase times to correctly set the third phase
                    holder.put(DataElementKey.SIGNAL_PHASE, new PhaseDataElement(movement.getCurrentStateAsEnum()));
                    holder.put(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE, new DoubleDataElement(movement.getMaxTimeRemaining()));
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
