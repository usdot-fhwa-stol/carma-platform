package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.IConsumerInitializer;
import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.appcommon.*;
import gov.dot.fhwa.saxton.glidepath.appcommon.utils.ConversionUtils;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.spat.SpatMessage;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Skeleton AsdConsumer
 */
public class TestAsdConsumer implements IConsumerTask {

    private ILogger logger = LoggerManager.getLogger(TestAsdConsumer.class);

    private IConsumerInitializer initializer = new TestAsdConsumerInitializer();

    private int iter_ = 0;
    private double currentDistance = 300.00;

    private int spatCountdown = 450;
    private final static int SIGNAL_DURATION = 150;

    private SignalPhase currentSignalPhase = SignalPhase.GREEN;

    private int currentLane = 12;

    public TestAsdConsumer()   {
        // left blank
    }

    // provide flexibility to init with a failing initializer
    public TestAsdConsumer(IConsumerInitializer initializer)   {
        this.initializer = initializer;
    }

    @Override
    public boolean initialize() {
        logger.debug("CON", "Initializing TestAsdConsumer");
        return true;
    }

    @Override
    public void terminate() {
        logger.debug("CON", "Terminating TestAsdConsumer");
    }

    public IConsumerInitializer getInitializer()   {
        return initializer;
    }

    public DataElementHolder call() throws IOException {
        DateTime startTime = new DateTime();

        if (spatCountdown == 0)   {
            if (currentSignalPhase.equals(SignalPhase.RED))   {
                currentSignalPhase = SignalPhase.GREEN;
                spatCountdown = SIGNAL_DURATION;
                currentDistance = 300.00;
            }
            else if (currentSignalPhase.equals(SignalPhase.GREEN))   {
                currentSignalPhase = SignalPhase.YELLOW;
                spatCountdown = 50;
            }
            else   {
                currentSignalPhase = SignalPhase.RED;
                spatCountdown = SIGNAL_DURATION;
                currentDistance = 0;
            }
        }

        PhaseDataElement ph = new PhaseDataElement(currentSignalPhase);

        //double distance = Math.max(0.0, 190.0 - 0.1*iter_*6.27); //simulating constant speed of 6.27 m/s
        double distance = ConversionUtils.getInstance().feetToMeters(currentDistance);
        currentDistance -= 1.00;
        DoubleDataElement dist = new DoubleDataElement(distance);

        //double time = Math.max(0.0, 35.0 - 0.1*iter_); //simulating time count-down to phase change
        int iTime = spatCountdown/10;
        double time = iTime;
        DoubleDataElement timeNextPhase = new DoubleDataElement(time);

        double timeThird = Math.max(35.0, 70.0 - 0.1*iter_);
        DoubleDataElement timeThirdPhase = new DoubleDataElement(timeThird);

        DataElementHolder rtn = new DataElementHolder();
        SpatMessage spat = new SpatMessage();
        List<IAsdMessage> list  = new ArrayList<IAsdMessage>();
        list.add(spat);
        IAsdListDataElement le = new IAsdListDataElement(list);
        rtn.put(DataElementKey.SPAT_LIST, le);

        /*****
        if (iter_ >= 200 && iter_ <= 230)   {
            // TESTING ONLY omit spat for a period of time, comment these out to induce spat error
            rtn.put(DataElementKey.SIGNAL_PHASE, ph);
            rtn.put(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE, timeNextPhase);
            rtn.put(DataElementKey.SIGNAL_TIME_TO_THIRD_PHASE, timeThirdPhase);
        }
        else   {
            rtn.put(DataElementKey.SIGNAL_PHASE, ph);
            rtn.put(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE, timeNextPhase);
            rtn.put(DataElementKey.SIGNAL_TIME_TO_THIRD_PHASE, timeThirdPhase);
        }
         *****/

        rtn.put(DataElementKey.DIST_TO_STOP_BAR, dist);

        // add lane id
        rtn.put(DataElementKey.LANE_ID, new IntDataElement(currentLane));

        ++iter_;

        spatCountdown -= 1;

        Duration duration = new Duration(startTime, new DateTime());
        rtn.put(DataElementKey.CYCLE_SPAT, new IntDataElement((int) duration.getMillis()));
        rtn.put(DataElementKey.CYCLE_MAP, new IntDataElement((int) duration.getMillis()));

        return rtn;
    }

}