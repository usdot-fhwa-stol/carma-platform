package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testutils;


import gov.dot.fhwa.saxton.glidepath.dvi.domain.MotionStatus;
import gov.dot.fhwa.saxton.glidepath.appcommon.*;

import static gov.dot.fhwa.saxton.glidepath.appcommon.DataElementKey.*;

public class TestUtils {

    /**
     * Create a non-full DataElementHolder with a missing element based on provided missingKey
     *
     * @param missingKey
     * @return DataElementHolder missing one element
     */
    public static DataElementHolder createDataElementHolder(DataElementKey missingKey)   {
        DataElementHolder holder = new DataElementHolder();

        DataElementKey[] doubles = {
                SPEED, SPEED_COMMAND, OPERATING_SPEED, SMOOTHED_SPEED, ACCELERATION, JERK, LATITUDE, LONGITUDE,
                DIST_TO_STOP_BAR, SIGNAL_TIME_TO_NEXT_PHASE, SIGNAL_TIME_TO_THIRD_PHASE, CYCLE_GPS,
                CYCLE_MAP, LATITUDE, CYCLE_SPAT, CYCLE_XGV, CYCLE_EAD
        };

        int i = 1;
        for (DataElementKey key : doubles)   {
            double iDouble = i++;
            DoubleDataElement doubleElement = new DoubleDataElement(iDouble);
            if (key != missingKey)   {
                holder.put(key, doubleElement);
            }
        }

        holder.put(DataElementKey.STATUS_MESSAGE, new StringBufferDataElement(new StringBuffer()));

        SignalPhase phase = SignalPhase.GREEN;
        PhaseDataElement phaseElement = new PhaseDataElement(phase);
        if (SIGNAL_PHASE != missingKey)   {
            holder.put(SIGNAL_PHASE, phaseElement);
        }

        DataElementHolder xgvHolder = XgvUtils.createXgvStatus();
        if (XGV_STATUS != missingKey)   {
            holder.putAll(xgvHolder);
        }

        MotionStatus motionStatus = MotionStatus.Speeding_Up;
        MotionStatusDataElement motionStatusElement = new MotionStatusDataElement(motionStatus);
        if (MOTION_STATUS != missingKey)   {
            holder.put(MOTION_STATUS, motionStatusElement);
        }

        return holder;
    }

    /**
     * Create a full DataElementHolder
     * @return DataElementHolder
     */
    public static DataElementHolder createFullDataElementHolder()   {
        return createDataElementHolder(null);
    }
}
