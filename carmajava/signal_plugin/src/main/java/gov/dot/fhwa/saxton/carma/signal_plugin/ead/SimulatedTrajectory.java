package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.*;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.joda.time.DateTime;
import org.joda.time.Duration;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.util.Iterator;

public class SimulatedTrajectory implements ITrajectory {
	
	
	// DOES NOT USE THE ACTUAL EAD ALGORITHM SO WE DON'T HAVE TO WORRY ABOUT A JNI CONNECTION
	//
	// The getSpeedCommand() performs the real input data validation here, even though that data isn't
	// being used to calculate a speed command.
	// As with the real EAD algorithm, this wrapper doesn't use the CSV file until the vehicle has
	// achieved the desired operating speed.  At that point it will start reading speed commands at
	// each time step until the file is consumed.  At that point it will continue to issue the
	// last command on the file indefinitely, until the stop bar has been crossed.
	//
	// Note: This class was built from the current state of the EadWrapper class on 12/16/14, which
	// is very close to completion at that time.  There is no intention to maintain currency if
	// that class gets further modified.
	
	
	/** Reads a CSV file full of speed commands to simulate the EAD's behavior.
	 * Note that it doesn't take into account realistic distance to stop bar.
	 * @throws Exception 
	 */
	public SimulatedTrajectory() throws Exception{
		
		//read a trajectory from a data file
		IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
		String filename = config.getProperty("ead.trajectoryfile");
		timeStepSize_ = (long)config.getPeriodicDelay();

		//if a filename is specified then use the file
		if (filename.length() > 0) {
	        try  {
	            File csv = new File(filename);
	            csvParser_ = CSVParser.parse(csv, Charset.forName("UTF-8"), CSVFormat.RFC4180);
	            iter_ = csvParser_.iterator();
	            log_.warnf("****", "1. Reading trajectory from %s", filename);
	        } catch (IOException e) {
	        	log_.errorf("****", "Cannot open CSV test file %s", filename);
	        	throw e;
	        }

	    //else prepare to use the EAD library
		}else {
			log_.error("****", "No trajectory file specified.");
			throw new Exception("Need to specify an ead.trajectoryfile in dvi.properties.");
		}
		
		prevCmd_ = 0.0;
		operSpeedAchieved_ = false;
		firstTimeOperSpeed_ = true;
	}
	
	/**config param ead.trajectoryfile not empty : read command from specified file
	 * 
	 * @param state contains current SPEED, OPERATING_SPEED, DIST_TO_STOP_BAR, SIGNAL_PHASE, 
	 *        SIGNAL_TIME_TO_NEXT_PHASE, SIGNAL_TIME_TO_THIRD_PHASE elements
	 * 
	 * @return DataElementHolder containint SPEED_COMMAND
	 * @throws Exception 
	 */
	public DataElementHolder getSpeedCommand(DataElementHolder state) throws Exception {
		long entryTime = System.currentTimeMillis();

		//extract input data and check that all values are from the same time step (don't check operating speed
		// because it is more of a manual input and not calculated every time step).
		DoubleDataElement curSpeed = (DoubleDataElement)state.get(DataElementKey.SPEED);
		DoubleDataElement operSpeed = (DoubleDataElement)state.get(DataElementKey.OPERATING_SPEED);
		DoubleDataElement dist = (DoubleDataElement)state.get(DataElementKey.DIST_TO_STOP_BAR);
		PhaseDataElement phase = (PhaseDataElement)state.get(DataElementKey.SIGNAL_PHASE);
		DoubleDataElement time1 = (DoubleDataElement)state.get(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE);
		DoubleDataElement time2 = (DoubleDataElement)state.get(DataElementKey.SIGNAL_TIME_TO_THIRD_PHASE);
		long oldestTime = curSpeed.timeStamp();
		if (dist.timeStamp() < oldestTime) oldestTime = dist.timeStamp();
		if (phase.timeStamp() < oldestTime) oldestTime = phase.timeStamp();
		if (time1.timeStamp() < oldestTime) oldestTime = time1.timeStamp();
		if (time2.timeStamp() < oldestTime) oldestTime = time2.timeStamp();
		if ((entryTime - oldestTime) > 0.9*timeStepSize_) { //allow time for this method to execute within the timestep
			log_.errorf("****", "EadWrapper.getSpeedCommand detects stale input data. curTime = %d, oldestTime = %d, timeStepSize_ = %d",
						entryTime, oldestTime, timeStepSize_);
			log_.errorf("", "    cur speed is        %5d ms old", entryTime-curSpeed.timeStamp());
			log_.errorf("", "    distance is         %5d ms old", entryTime-dist.timeStamp());
			log_.errorf("", "    phase is            %5d ms old", entryTime-phase.timeStamp());
			log_.errorf("", "    time next phase is  %5d ms old", entryTime-time1.timeStamp());
			log_.errorf("", "    time third phase is %5d ms old", entryTime-time2.timeStamp());
			throw new ObsoleteDataException("One or more data elements is older than one time step. See log for details.");
		}
		
		double cmd = 0.0; //the output speed command, m/s

		//if signal is red or yellow but the vehicle is past the stop bar (maybe a positional error?) then
		if (phase.value() != SignalPhase.GREEN  &&  dist.value() <= 0.0) {
		
			//ensure the vehicle remains stopped
			cmd = 0.0;
			log_.warnf("EAD", "Beyond stop bar at red light. Distance = %.2f m, speed = %.2f m/s", dist.value(), curSpeed.value());
		
		//else if we have never achieved user's desired operating speed and the current speed is still below the operating speed then\
		// (we are either in the initial acceleration phase or departing past the intersections after a complete stop)
		}else if (!operSpeedAchieved_  &&  (operSpeed.value() - curSpeed.value()) > 0.1) {
			
			//just set the commanded speed to the desired speed; the XGV will handle the acceleration okay
			cmd = operSpeed.value();
			log_.infof("EAD", "3. Operating speed not yet achieved. Oper speed = %.2f m/s,  cur speed = %.2f m/s", operSpeed.value(), curSpeed.value());
		
		//else (we can invoke the EAD algorithm)
		}else {
			
			//indicate that we have achieved operating speed (we touched it for at least one time step)
			operSpeedAchieved_ = true;
			if (firstTimeOperSpeed_) {
				firstTimeOperSpeed_ = false;
				log_.infof("EAD", "4. Achieved operating speed of %.2f m/s", curSpeed.value());
			}
		
			//if we are using a trajectory file for simulation then
			if (csvParser_ != null) {
				try  {
					CSVRecord rec = iter_.next();
					//double time = Double.parseDouble(rec.get(0).trim()); //don't need this now, but it's in the data file
					cmd = Double.parseDouble(rec.get(1).trim());
				} catch (Exception e) {
					cmd = prevCmd_;
					log_.error("EAD", "No more commands in trajectory file. Using last command in file.");
				}
			}else {
				log_.error("****", "csvParser is null.");
				throw new Exception("csvParser is null");
			}
		}
		
		//assemble the output
		prevCmd_ = cmd;
		DoubleDataElement speedCmd = new DoubleDataElement(cmd);
		DataElementHolder rtn = new DataElementHolder();
		rtn.put(DataElementKey.SPEED_COMMAND, speedCmd);
		log_.debugf("", "7. EadWrapper.getSpeedCommand exiting. Commanded speed is %.2f m/s. Method time = %d ms.", 
				cmd, System.currentTimeMillis()-entryTime);

        Duration duration = new Duration(new DateTime(entryTime), new DateTime());
        rtn.put(DataElementKey.CYCLE_EAD, new IntDataElement((int) duration.getMillis()));

        return rtn;
	}

	public void close() {
		//do nothing - this is only for using the EADlib
	}
	
	public void engage() {
		//not needed in simulation
	}

	public boolean isStopConfirmed() {
		return false; //bogus for simulation
	}
	
	
	////////// internal attributes
	
	private static ILogger		log_ = LoggerManager.getLogger(Trajectory.class);
	private double				prevCmd_;			//speed command from the previous time step
	private long				timeStepSize_;		//duration of a single time step, ms
	private boolean				operSpeedAchieved_;	//soft latch; have we achieved operating speed at some point since vehicle started moving?
	private boolean				firstTimeOperSpeed_;//is this the first time step that operating speed has been achieved?
	private CSVParser			csvParser_;
	private Iterator<CSVRecord>	iter_;
}
