package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.io.RandomAccessFile;

import static gov.dot.fhwa.saxton.glidepath.dvi.domain.GlidepathState.*;

/**
 * Maintains and transitions to glidepath states
 *
 * Order:
 * START
 * SETUP
 * STANDBY
 * ENGAGED
 * DISENGAGED
 */
public class GlidepathStateModel {

    private GlidepathState glidepathState = STARTUP;

    private ILogger logger = LoggerManager.getLogger(GlidepathStateModel.class);

    private static String statefile;

    private GlidepathStateModel() {
    }

    private static class GlidepathStateModelHolder  {
        private static final GlidepathStateModel _instance = new GlidepathStateModel();
    }

    public static GlidepathStateModel getInstance()
    {
        return GlidepathStateModelHolder._instance;
    }


    /**
     * Loads the member statefile variable with location of persisted state information for apache
     */
    public static void getConfiguredStateFile()
    {
        statefile = (GlidepathApplicationContext.getInstance().getAppConfig()).getProperty("dvi.statefile");
        if (statefile == null || statefile.length() == 0)    {
            statefile = "/var/wwww/html/data/state.dat";
        }
    }


    /**
     * Get the current glidepath state
     *
     * @return GlidepathState
     */
    public synchronized GlidepathState getState()   {
        return glidepathState;
    }


    /**
     * Changes to desired state if allowed
     *
     * @param newState
     * @param holder
     * @return
     */
    public synchronized boolean changeState(GlidepathState newState, DataElementHolder holder)    {
        boolean bReturn = false;

        switch (newState)   {

            case STARTUP:
                bReturn = true;
                break;

            case SETUP:
                if (glidepathState.equals(STARTUP) || glidepathState.equals(STANDBY))   {
                    bReturn = true;
                }
                break;

            case STANDBY:
                // stop device threads
                if (glidepathState.equals(SETUP))   {
                    bReturn = true;
                }

                else if (glidepathState.equals(ENGAGED))   {
                    bReturn = true;
                }

                else if (glidepathState.equals(DISENGAGED))   {
                    bReturn = true;
                }
                break;

            case DISENGAGED:
                if (glidepathState.equals(STANDBY))   {
                    bReturn = true;
                }

                else if (glidepathState.equals(ENGAGED))   {
                    bReturn = true;
                }
                break;

            case ENGAGED:
                if (glidepathState.equals(STANDBY))   {
                    bReturn = true;
                }

                else if (glidepathState.equals(DISENGAGED))   {
                    bReturn = true;
                }
                break;

            case FATAL:
                bReturn = true;
                break;

        }

        if (bReturn)   {
            glidepathState = newState;
            writeToFile();
            logger.infof("STATE", "Changing glidepath state to %s.", glidepathState.toString());
        }

        return bReturn;
    }

    /**
     * Writes the state value to the state file used by apache
     */
    private void writeToFile()   {
        try   {
            if (statefile == null)   {
                getConfiguredStateFile();
            }

            RandomAccessFile apacheStateFile = new RandomAccessFile(statefile, "rwd");
            apacheStateFile.setLength(0);
            apacheStateFile.seek(0);
            apacheStateFile.writeBytes(glidepathState.toString());
            apacheStateFile.close();
            logger.debug("STATE", "Updated state file with current status: " + glidepathState.toString());
        }
        catch(Exception e)   {
            logger.warn("STATE", "Error writing glidepath state to common state file: " + e.getMessage());
        }
    }

    /**
     * Reset state to STARTUP
     */
    public void reset()   {
        glidepathState = STARTUP;
    }
}
