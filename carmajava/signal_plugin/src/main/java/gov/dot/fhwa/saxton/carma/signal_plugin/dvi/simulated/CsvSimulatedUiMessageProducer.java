package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase;

import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.DviUIMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.GlidepathState;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.GlidepathStateModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.MotionStatus;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/**
 * CsvSimulatedUiMessageProducer
 *
 * A simulator to periodically provide DviUIMessages to the DviExecutorService
 *
 * Loads CSV data from provided file (testdata/uiMessages.csv
 *
 */
public class CsvSimulatedUiMessageProducer {

    private static Logger logger = LoggerFactory.getLogger(CsvSimulatedUiMessageProducer.class);

    private String uiMessageFile;

    private List<DviUIMessage> uiMessages = new ArrayList<DviUIMessage>();

    private int currentIndex = 0;
    private int maxIndex = 0;

    public CsvSimulatedUiMessageProducer(String uiMessageFile)   {
        this.uiMessageFile = uiMessageFile;
    }

    public DviUIMessage getUiMessage()   {

        DviUIMessage uiMessage;

        if (uiMessages.isEmpty())  {
            uiMessage = new DviUIMessage();
        }
        else   {
            uiMessage = uiMessages.get(currentIndex);
            currentIndex += 1;
            if (currentIndex >= maxIndex)  {
                currentIndex = 0;
            }
        }

        return uiMessage;
    }



    protected void load()   {
        try  {
            File csvData = new File(uiMessageFile);
            CSVParser parser = CSVParser.parse(csvData, Charset.forName("UTF-8"), CSVFormat.RFC4180);
            for (CSVRecord csvRecord : parser) {

                //GlidepathState glidepathState = GlidepathState.valueOf(csvRecord.get(0).trim());
                GlidepathState glidepathState = GlidepathStateModel.getInstance().getState();

                SignalPhase signalPhase = SignalPhase.valueOf(csvRecord.get(1).trim());

                double timeNextPhase = Double.parseDouble(csvRecord.get(2).trim());
                double timeThirdPhase = Double.parseDouble(csvRecord.get(3).trim());

                MotionStatus motionStatus = MotionStatus.valueOf(csvRecord.get(4).trim());

                double speed = Double.parseDouble(csvRecord.get(5).trim());
                double latitude = Double.parseDouble(csvRecord.get(6).trim());

                double longitude = Double.parseDouble(csvRecord.get(7).trim());
                double distanceToStopBar = Double.parseDouble(csvRecord.get(8).trim());
                double targetSpeed = Double.parseDouble(csvRecord.get(9).trim());

                DviUIMessage uiMessage = new DviUIMessage(glidepathState, signalPhase, timeNextPhase, timeThirdPhase,
                        motionStatus, speed, targetSpeed, latitude, longitude, distanceToStopBar, "", new Date().getTime(), false, 11, 1001, true);

                uiMessages.add(uiMessage);
            }
        }
        catch(Exception e)   {
            logger.error("Error loading CSV DviUIMessage data. ", e);
        }

        maxIndex = uiMessages.size();
    }

    public int getMaxIndex()   {
        return maxIndex;
    }

}


