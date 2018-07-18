package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * DviParameters
 *
 * Singleton holding parameters that support DVI state
 *
 */
public class DviParameters {
    private static Logger logger = LoggerFactory.getLogger(DviParameters.class);

    /**
     * speeds currently mph, we convert to metric for non-UI components
     */
    private double operatingSpeed = (GlidepathApplicationContext.getInstance().getAppConfig()).getDefaultSpeed();            // default operating speed...Web UI update
    private double maximumSpeed;
    private String logFileName;                    // generated log name based on when eco driver started, operating
                                                   //  speed, and spat status
    private String logSpeedCsv;                    // ditto above, for speeds.csv


    private DviParameters() {
        // in case we decide to never go into EcoDrive, load with default values so we can at least roll the logs
        this.setLogFileNames("NoEcoDrive");
    }

    private static class DviParametersHolder  {
        private static final DviParameters _instance = new DviParameters();
    }

    public static DviParameters getInstance()
    {
        return DviParametersHolder._instance;
    }

    public double getOperatingSpeed()   {
        return operatingSpeed;
    }

    public void setOperatingSpeed(double speed)   {
        this.operatingSpeed = speed;
    }

    public double getMaximumSpeed()   {
        return maximumSpeed;
    }

    public void setMaximumSpeed(double speed)   {
        this.maximumSpeed = speed;
    }

    public void setLogFileNames(String logFilePrefix)    {

        DateTime now = new DateTime();
        DateTimeFormatter fmt = DateTimeFormat.forPattern("yyyyMMdd.HHmmss");
        String strNow = fmt.print(now);

        logFileName = strNow + "." + logFilePrefix + ".log";
        logSpeedCsv = "speeds." + strNow + "." + logFilePrefix + ".csv";
    }


    public String getLogFileName()   {
        return logFileName;
    }

    public String getLogSpeedCsv()   {
        return logSpeedCsv;
    }

}