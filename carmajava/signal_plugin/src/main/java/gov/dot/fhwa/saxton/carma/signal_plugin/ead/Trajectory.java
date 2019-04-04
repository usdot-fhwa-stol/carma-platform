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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.*;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ANAStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.io.IOException;
import java.util.*;

/**
 * This class describes the speed trajectory for the Glidepath vehicle in any given situation.
 * Its primary purpose is to invoke the EAD algorithm, but also manages alternative methods of
 * determining speed commands (which may replace or override the EAD in any given time step), 
 * ensuring that the vehicle is stopped
 * at a red light, and providing emergency override (failsafe) commands to prevent running of
 * a red light.
 */
public class Trajectory implements ITrajectory {

	/**
	 * Default constructor that will generate its own EAD model to be used if the @param trajectoryfile
	 * is not specified.
	 * @throws Exception
	 */
	public Trajectory() throws Exception {
		IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
		assert(config != null);

		//get the desired EAD variant from the config file and instantiate it
		String eadClass = config.getProperty("ead.modelclass");
		if (eadClass == null) {
			eadClass = "default";
		}
		IEad ead;
		try {
			ead = EadFactory.newInstance(eadClass);	
		} catch (InstantiationException e) {
			log_.errorf("TRAJ", "Could not instantiate the EAD model %s", eadClass);
			throw new Exception("Could not instantiate an EAD model.");
		} catch (IllegalAccessException e) {
			log_.errorf("TRAJ", "Could not instantiate the EAD model %s", eadClass);
			throw new Exception("Could not instantiate an EAD model.");
		}
		if(ead == null) {
			log_.errorf("TRAJ", "Could not instantiate the EAD model %s", eadClass);
			throw new Exception("Could not instantiate an EAD model.");
		}
		
		log_.debug("TRAJ", "Ready to construct EAD object.");

		constructObject(ead);
	}

	/**
	 * Constructor that allows the parent to inject a particular EAD model
	 * @param eadModel
	 * @throws Exception
	 */
	public Trajectory(IEad eadModel) throws Exception {
		constructObject(eadModel);
	}

	/** Creates all of the persistent attributes of the new Trajectory object.
	 * Connects to the EAD algorithm library and initializes it for first use.
	 * @param eadModel - the EAD object that will be used to compute the trajectory
	 * @throws IOException if any of the needed config parameters cannot be read
	 * @throws Exception if there is an error initializing or executing the EAD model
	 */
	protected void constructObject(IEad eadModel) throws Exception {
		//Get application context
		IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
		assert(config != null);
		timeStepSize_ = (long)config.getPeriodicDelay();
		
		//determine if we will be using the timeouts or allowed to run slow (for testing)
		respectTimeouts_ = config.getBooleanValue("performancechecks");
		log_.infof("TRAJ", "time step = %d ms, respecting timeouts = %b", timeStepSize_, respectTimeouts_);

		//get constraint parameters from the config file
		speedLimit_ = config.getMaximumSpeed(0.0)/ Constants.MPS_TO_MPH; //max speed is the only parameter in the config file that is in English units!
		maxJerk_ = config.getDoubleValue("maximumJerk");
		accelLimiter_ = config.getBooleanValue("ead.accelLimiter");
		jerkLimiter_ = config.getBooleanValue("ead.jerkLimiter");
		log_.infof("TRAJ", "speedLimit = %.2f, maxJerk = %.2f", speedLimit_, maxJerk_);
		log_.infof("TRAJ", "accel limiter = %b, jerk limiter = %b", accelLimiter_, jerkLimiter_);

		//get the list of intersections that we will be paying attention to
		String tmpList = config.getProperty("asd.intersections");
		if (tmpList != null) {
			List<String> items = Arrays.asList(tmpList.split("\\s*,\\s*"));
			int number = items.size();
			intersectionIds_ = new int[number];
			for (int i = 0;  i < number;  ++i) {
				intersectionIds_[i] = Integer.parseInt(items.get(i));
			}
			log_.infof("TRAJ", "Using intersections: %s", tmpList);
		}else {
			log_.warn("TRAJ", "No intersections specified for the ASD.");
		}

		//get the max allowable spat error count
		maxSpatErrors_ = config.getIntValue("ead.max.spat.errors");
		log_.infof("TRAJ", "Max spat errors = %d", maxSpatErrors_);

		//get failsafe parameters
		allowFailSafe_			= config.getBooleanValue("ead.failsafe.on");
		failSafeDistBuf_		= config.getDoubleValue("ead.failsafe.distance.buffer");
		failSafeResponseLag_	= config.getDoubleValue("ead.response.lag");
		log_.infof("TRAJ", "allowFailSafe = %b, failSafeDistBuf = %.2f, failSafeResponseLag = %.2f", allowFailSafe_, failSafeDistBuf_, failSafeResponseLag_);
		maxCmdAdj_		= config.getDoubleValue("ead.maxcmdadj");
		cmdAccelGain_	= config.getDoubleValue("ead.cmdaccelgain");
		cmdSpeedGain_	= config.getDoubleValue("ead.cmdspeedgain");
		cmdBias_		= config.getDoubleValue("ead.cmdbias");
		log_.infof("TRAJ", "maxCmdAdj = %.2f, cmdAccelGain = %.4f, cmdSpeedGain = %.4f, cmdBias = %.4f",
					maxCmdAdj_, cmdAccelGain_, cmdSpeedGain_, cmdBias_);
		
		ead_ = eadModel;

		//pass config parameters to the EAD library
		try {
			ANAStarSolver solver = new ANAStarSolver();
			solver.setMaxPlanningTimeMS(200);
			ead_.initialize(timeStepSize_, solver);
			//ead_.initialize(timeStepSize_, new AStarSolver());
		} catch (Exception e) {
			log_.errorf("TRAJ", "Exception thrown by EAD library initialize(). maxJerk = %f, speedLimit = %f",
					maxJerk_, speedLimit_);
			throw e;
		}
		
		log_.infof("TRAJ", "2. EADlib initialized. speedLimit = %.2f, maxJerk = %.2f",
				speedLimit_, maxJerk_);
		
		//initialize other members
		curSpeed_ = 0.0;
		curAccel_ = 0.0;
		stopConfirmed_ = false;
		timeRemaining_ = 0.0;
	}

	/**
	 * Invokes the native EADlib shutdown function
	 */
	public void close() {
		//deprecated
	}
	
	/**
	 * Indicates that this software is now authorized to control the vehicle's speed
	 */
	public void engage() {
	//	motionAuthorized_ = true;
		log_.info("TRAJ", "///// Driver engaged automatic control.");
	}

	/**
	 * confirmed stop (after first motion) : true
	 * still in motion : false
     */
	public boolean isStopConfirmed() {
		return stopConfirmed_;
	}

	/**
	 * Returns a list of known intersections sorted from nearest to farthest
	 * 
	 * @return A list of IntersectionData sorted from nearest to farthest
	 */
	public List<IntersectionData> getSortedIntersections() {
		return new ArrayList<>(intersectionManager_.getSortedIntersections());
	}
	

@Override	public List<Node> plan(DataElementHolder stateData) throws Exception {
		long entryTime = System.currentTimeMillis();


		////////// EXTRACT & VALIDATE INPUT STATE (except SPAT)


		DoubleDataElement curSpeedElement;
		DoubleDataElement curAccelElement;
		DoubleDataElement operSpeedElem;
		DoubleDataElement vehicleLat;
		DoubleDataElement vehicleLon;
		List<IntersectionData> inputIntersections = new ArrayList<>();
		double operSpeed;
		try {
			//convert all input data from the incoming state vector; this will include all intersections now in view
			if (stateData == null) {
				log_.error("TRAJ", "plan - input stateData is null.");
				throw new Exception("No state data input to getSpeedCommand.");
			}
			curSpeedElement = (DoubleDataElement) stateData.get(DataElementKey.SMOOTHED_SPEED);
			curAccelElement = (DoubleDataElement) stateData.get(DataElementKey.ACCELERATION);
			operSpeedElem = (DoubleDataElement) stateData.get(DataElementKey.OPERATING_SPEED);
			vehicleLat = (DoubleDataElement) stateData.get(DataElementKey.LATITUDE);
			vehicleLon = (DoubleDataElement) stateData.get(DataElementKey.LONGITUDE);
			IntersectionCollectionDataElement icde =
					(IntersectionCollectionDataElement) stateData.get(DataElementKey.INTERSECTION_COLLECTION);
			if (icde != null) {
				inputIntersections = icde.value().getIntersections();
			}

			//check that all critical elements are present - will throw exception if missing
			validateElement(curSpeedElement, "curSpeedElement", true);
			validateElement(curAccelElement, "curAccelElement", true);
			validateElement(operSpeedElem, "operSpeed", true);
			validateElement(vehicleLat, "vehicleLat", true);
			validateElement(vehicleLon, "vehicleLon", true);

			// override the driver selected operating speed with one stored in the config file if it is valid (for testing)
			operSpeed = operSpeedElem.value();

			//track how old the critical input elements are
			long oldestTime;
			if (respectTimeouts_) {
				oldestTime = curSpeedElement.timeStamp();
				if (curAccelElement.timeStamp() < oldestTime) oldestTime = curAccelElement.timeStamp();
				if (vehicleLat.timeStamp() < oldestTime) oldestTime = vehicleLat.timeStamp();
				if (vehicleLon.timeStamp() < oldestTime) oldestTime = vehicleLon.timeStamp();
				if ((entryTime - oldestTime) > 0.9 * timeStepSize_) { //allow time for this method to execute within the time step
					log_.warnf("TRAJ", "plan detects stale input data. curTime = %d, oldestTime = %d, timeStepSize_ = %d",
							entryTime, oldestTime, timeStepSize_);					
					log_.warnf("TRAJ", "    speed is            %5d ms old", entryTime - curSpeedElement.timeStamp());
					log_.warnf("TRAJ", "    accel is            %5d ms old", entryTime - curAccelElement.timeStamp());
					log_.warnf("TRAJ", "    latitude is         %5d ms old", entryTime - vehicleLat.timeStamp());
					log_.warnf("TRAJ", "    longitude is        %5d ms old", entryTime - vehicleLon.timeStamp());
				}
			}			
			curSpeed_ = curSpeedElement.value(); //this is the smoothed value
			curAccel_ = curAccelElement.value(); //smoothed
		}catch (Exception e) {
			log_.error("TRAJ", "Unknown exception trapped in input processing: " + e.toString());
			throw e;
		}


		DoubleDataElement startTime = (DoubleDataElement) stateData.get(DataElementKey.PLANNING_START_TIME);
		DoubleDataElement startDowntrack = (DoubleDataElement) stateData.get(DataElementKey.PLANNING_START_DOWNTRACK);

		////////// UNDERSTAND THE INTERSECTION GEOMETRY & WHERE WE FIT IN


		double dtsb;
		double lat = vehicleLat.value();
		double lon = vehicleLon.value();
		try {
			//update internal record of intersections in view, sorted by distance from the vehicle (nearest first)
			// ASSUMES all intersections within view are ones on our route (we will be traversing them)
			Location vehicleLoc = new Location(lat, lon);
			dtsb = intersectionManager_.updateIntersections(inputIntersections, vehicleLoc);

			log_.debug("TRAJ", "plan returned from call to updateIntersections");

		}catch (Exception e) {
			log_.error("TRAJ", "plan detected exception in intersection or spat calcs: " + e.toString());
			throw e;
		}


		////////// RUN EAD TO GENERATE NEW PLAN

		List<Node> path;

		//get speed command from EAD (assume it will handle position in stop box w/red light)
		try {
			log_.info("TrafficSignalPlugin", this.getSortedIntersections().toString());
			path = ead_.plan(curSpeed_, operSpeed, this.getSortedIntersections(), startTime.value(), startDowntrack.value());
		}catch (Exception e) {
			log_.errorf("TRAJ", "Exception trapped by EAD algo: " + e.toString() +
			"\n    Too many errors...rethrowing. " +
			"curSpeed = %.2f, operSpeed = %.2f, dtsb = %.2f, timeRemaining = %.2f",
				curSpeed_, operSpeed, dtsb, timeRemaining_);
			throw e;
		}

		log_.debug("TRAJ", "plan completed executing current state.");
		return path;
	}


	//////////////////
	// member elements
	//////////////////


	/**
	 * if element is null, then write a log warning.
	 * @throws Exception if the input object is null
	 */
	private void validateElement(Object elem, String name, boolean logIt) throws Exception {
		if (elem == null) {
			if (logIt) log_.warnf("TRAJ", "Critical input state element is null:  %s", name);
			throw new Exception("Critical input state element is null: " + name);
		}
	}

	/**
	 * Refreshes the internal list of intersections in view with the latest input data.
	 * NOTE that each intersection in view will normally give us a spat every time step (not guaranteed), but
	 * will only give us a map occasionally. Therefore, we need to remember the maps when they arrive.
	 * NOTE that, for failsafe and state transition purposes, we are not considered in EAD mode (in an intersection)
	 * unless we are on an approach lane to the nearest intersections. It is possible that a more distant
	 * intersections has a farther reaching map and we are on an approach lane to that one already but not on the
	 * map for the nearest intersection yet. In that case we should still consider ourselves not on an intersection
	 *
	 * @param inputIntersections - list of intersections sensed by the vehicle in this time step
	 * @param vehicleLoc - current location of the vehicle
	 * @return distance to stop bar of the nearest intersection
	 */
	public double updateIntersections(List<IntersectionData> inputIntersections, Location vehicleLoc) throws Exception {

		return intersectionManager_.updateIntersections(inputIntersections, vehicleLoc);
	}
	
	private long				timeStepSize_;		//duration of a single time step, ms
	private boolean				stopConfirmed_;		//are we at a complete stop?
	private boolean				respectTimeouts_;	//should we honor the established timeouts?
	private int[]				intersectionIds_;	//array of IDs of intersections that we will pay attention to
	private int					maxSpatErrors_;		//max allowable number of consecutive spat data errors
	private double				speedLimit_;		//upper limit allowed for speed, m/s
	private double				maxJerk_;			//max allowed jerk, m/s^3
	private double				curSpeed_;			//current vehicle speed, m/s
	private double				curAccel_;			//current acceleration (smoothed), m/s^2
	private double				timeRemaining_;		//time remaining in the current signal phase, sec
	private IEad				ead_;				//the EAD model that computes the speed commands
	private boolean				accelLimiter_;		//is acceleration limiter used?
	private boolean				jerkLimiter_;		//is jerk limiter used?
	private double				maxCmdAdj_;			//difference (m/s) needed between current speed and command given to the XGV to get it to respond quickly
	private boolean				allowFailSafe_;		//does the user want failsafe logic involved?
	private double				failSafeDistBuf_;	//distance buffer that failsafe will subtract from stop bar location, meters
	private double				failSafeResponseLag_; //time that failsafe logic allows for the vehicle to respond to a command change, sec
	private double				cmdSpeedGain_;		//gain applied to speed difference for fail-safe calculations
	private double				cmdAccelGain_;		//gain applied to acceleration for fail-safe calculations
	private double				cmdBias_;			//bias applied to command premium for fail-safe calculations

	private static ILogger		log_ = LoggerManager.getLogger(Trajectory.class);

	private EadIntersectionManager intersectionManager_ = new EadIntersectionManager();
}
