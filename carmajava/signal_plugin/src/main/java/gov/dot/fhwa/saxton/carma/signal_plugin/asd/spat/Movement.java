package gov.dot.fhwa.saxton.glidepath.asd.spat;

import gov.dot.fhwa.saxton.glidepath.appcommon.SignalPhase;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.util.ArrayList;
import java.util.List;

/**
 * A SPAT movement
 *
 * Contains the list of LaneSet and the movements associated with these lanes (state, minTime, maxTime)
 *
 * User: ferenced
 * Date: 1/17/15
 * Time: 12:50 PM
 *
 */
public class Movement {
    private static ILogger logger =  LoggerManager.getLogger(Movement.class);

    List<LaneSet> laneSets = new ArrayList<LaneSet>();
    int currentState = 0;
    double minTimeRemaining = 0;
    double maxTimeRemaining = 0;

    public List<LaneSet> getLaneSets()   {
        return laneSets;
    }

    public void addLaneSet(LaneSet laneSet)   {
        laneSets.add(laneSet);
    }

    public void setCurrentState(int state)   {
        this.currentState = state;
    }

    public void setMinTimeRemaining(int timeRemaining)   {
        this.minTimeRemaining = (double) timeRemaining / 10;
    }

    public double getMinTimeRemaining()   {
        return minTimeRemaining;
    }

    public void setMaxTimeRemaining(int timeRemaining)   {
        this.maxTimeRemaining = (double) timeRemaining / 10;
    }

    public double getMaxTimeRemaining()   {
        return maxTimeRemaining;
    }

    /**
     * Returns a SignalPhase enum value of RED, YELLOW, or GREEN for the movement
     *
     * @return SignalPhase
     */
    public SignalPhase getCurrentStateAsEnum()   {
        SignalPhase phase = null;

        if ((currentState & 0x00000001)== 0x00000001)  {
            phase = SignalPhase.GREEN;
        }
        else if ((currentState & 0x00000002)== 0x00000002)  {
            phase = SignalPhase.YELLOW;
        }
        else if ((currentState & 0x00000004)== 0x00000004)  {
            phase = SignalPhase.RED;
        }
        else   {
            // we will set to green...frequently shows Dark 0x00000000 when in flashing mode
            phase = SignalPhase.GREEN;
        }

        return phase;
    }

    /**
     * String representation of exact phase of the signal for the movement per ICD
     *
     * @return
     */
    public String getCurrentState()
    {
        int i = currentState;
        String phase = "";
        if (i == 0x00000000)   {
            return "Dark";
        }

        if ((i & 0x00000001)==0x00000001)
            phase = phase + "Solid green ball/";
        if ((i & 0x00000002)==0x00000002)
            phase = phase + "Solid yellow ball/";
        if ((i & 0x00000004)==0x00000004)
            phase = phase + "Solid red ball/";
        if ((i & 0x00000008)==0x00000008)
            phase = phase + "Solid flashing ball/";
        if ((i & 0x00000010)==0x00000010)
            phase = phase + "Green left arrow/";
        if ((i & 0x00000020)==0x00000020)
            phase = phase + "Yellow left arrow/";
        if ((i & 0x00000040)==0x00000040)
            phase = phase + "Red left arrow/";
        if ((i & 0x00000080)==0x00000080)
            phase = phase + "Flashing left arrow/";
        if ((i & 0x00000100)==0x00000100)
            phase = phase + "Green right arrow/";
        if ((i & 0x00000200)==0x00000200)
            phase = phase + "Yellow right arrow/";
        if ((i & 0x00000400)==0x00000400)
            phase = phase + "Red right arrow/";
        if ((i & 0x00000800)==0x00000800)
            phase = phase + "Flashing right arrow/";
        if ((i & 0x00001000)==0x00001000)
            phase = phase + "Green straight arrow/";
        if ((i & 0x00002000)==0x00002000)
            phase = phase + "Yellow straight arrow/";
        if ((i & 0x00004000)==0x00004000)
            phase = phase + "Red straight arrow/";
        if ((i & 0x00008000)==0x00008000)
            phase = phase + "Flashing straight arrow/";
        if ((i & 0x00010000)==0x00010000)
            phase = phase + "Green soft left arrow/";
        if ((i & 0x00020000)==0x00020000)
            phase = phase + "Yellow soft left arrow/";
        if ((i & 0x00040000)==0x00040000)
            phase = phase + "Red soft left arrow/";
        if ((i & 0x00080000)==0x00080000)
            phase = phase + "Flashing soft left arrow/";
        if ((i & 0x00100000)==0x00100000)
            phase = phase + "Green soft right arrow/";
        if ((i & 0x00200000)==0x00200000)
            phase = phase + "Yellow soft right arrow/";
        if ((i & 0x00400000)==0x00400000)
            phase = phase + "Red soft right arrow/";
        if ((i & 0x00800000)==0x00800000)
            phase = phase + "Flashing soft right arrow/";
        if ((i & 0x01000000)==0x01000000)
            phase = phase + "Green U turn arrow/";
        if ((i & 0x02000000)==0x02000000)
            phase = phase + "Yellow U turn arrow/";
        if ((i & 0x04000000)==0x04000000)
            phase = phase + "Red U turn arrow/";
        if ((i & 0x08000000)==0x08000000)
            phase = phase + "Flashing U turn arrow/";

        return phase;
    }

}
