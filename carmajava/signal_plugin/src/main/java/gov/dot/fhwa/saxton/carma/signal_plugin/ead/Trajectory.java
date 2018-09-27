/*
 * Copyright (C) 2018 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.ISpatMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.io.IOException;
import java.util.*;

import static gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.SignalPhase.NONE;

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
		IEad ead = EadFactory.newInstance(eadClass);
		if (ead == null) {
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
			ead_.initialize(timeStepSize_, new AStarSolver());
		} catch (Exception e) {
			log_.errorf("TRAJ", "Exception thrown by EAD library initialize(). maxJerk = %f, speedLimit = %f",
					maxJerk_, speedLimit_);
			throw e;
		}
		
		log_.infof("TRAJ", "2. EADlib initialized. speedLimit = %.2f, maxJerk = %.2f",
				speedLimit_, maxJerk_);
		
		// Get operating speed override - if this is non-zero it will be used for OS regardless of user input! Intended for testing only!
		osOverride_ = 0.0;
		String tempStr = config.getProperty("ead.osOverride");
		if (tempStr != null) {
			double tempVal = Double.valueOf(tempStr);
			if (tempVal > 0.0  &&  tempVal < 35.0) {
				osOverride_ = tempVal;
				log_.warnf("TRAJ", "///// OS has been overridden with %.1f mph", tempVal);
			}
		}
		
		//initialize other members
		curSpeed_ = 0.0;
		curAccel_ = 0.0;
		stopConfirmed_ = false;
		phase_ = NONE;
		timeRemaining_ = 0.0;
		failSafeMode_ = false;
		numStepsAtZero_ = 0;
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
	
	// /**
	//  * 
	//  * NOTE: In this plugin this function does not return meaningful commands. Instead whole plans are ingested from the EadAStar object at a higher level
	//  * 
	//  * Computes the speed command, ID of nearest intersections in view and DTSB relative to that intersections
	//  * for the current time step based on various combinations of config parameters and
	//  * previous handling of intersections description data.
	//  *
	//  * (state contains valid MAP message || valid MAP previously received) &&
	//  * 		intersections geometry fully decomposed  &&  vehicle position is associated with a lane  &&
	//  * 			signal is red/yellow && (-W < DTSB < 0) : command = 0, computed DTSB
	//  * 			signal is green || (DTSB > 0) : speed command from EAD library, computed DTSB
	//  * 		intersections not fully decomposed  ||  vehicle cannot be associated with a single lane :
	//  * 			command = operating speed, DTSB = very large
	//  * 	    &&  no valid MAP has ever been received :
	//  * 			command = operating speed, DTSB = very large
	//  * 
	//  * Note: DTSB = distance to stop bar; in situations when vehicle can't associate an intersections and lane
	//  * 		(e.g. out of DSRC radio range of any intersections) then we still want the vehicle to operate under
	//  * 		automated control, so will set the speed command to the operating speed.
	//  * 		W = width of the intersection's stop box.
	//  * 
	//  * @param stateData contains current SPEED, OPERATING_SPEED, ACCELERATION, LATITUDE (vehicle), LONGITUDE (vehicle),
	//  *        list of known intersections (including MAP & SPAT data for each). It may
	//  *        also contain other elements.
	//  * 
	//  * @return DataElementHolder containing SPEED_COMMAND, SELECTED_INTERSECTION_ID, LANE_ID, DTSB, SIGNAL_PHASE/TIME
	//  *
	//  * @throws Exception for invalid input data or various computational anomalies
	//  */
	// public DataElementHolder getSpeedCommand(DataElementHolder stateData) throws Exception {
	// 	long entryTime = System.currentTimeMillis();


	// 	////////// EXTRACT & VALIDATE INPUT STATE (except SPAT)


	// 	DoubleDataElement curSpeedElement;
	// 	DoubleDataElement curAccelElement;
	// 	DoubleDataElement operSpeedElem;
	// 	DoubleDataElement vehicleLat;
	// 	DoubleDataElement vehicleLon;
	// 	List<IntersectionData> inputIntersections = new ArrayList<>();
	// 	double operSpeed;
	// 	try {
	// 		//convert all input data from the incoming state vector; this will include all intersections now in view
	// 		if (stateData == null) {
	// 			log_.error("TRAJ", "getSpeedCommand - input stateData is null.");
	// 			throw new Exception("No state data input to getSpeedCommand.");
	// 		}
	// 		curSpeedElement = (DoubleDataElement) stateData.get(DataElementKey.SMOOTHED_SPEED);
	// 		curAccelElement = (DoubleDataElement) stateData.get(DataElementKey.ACCELERATION);
	// 		operSpeedElem = (DoubleDataElement) stateData.get(DataElementKey.OPERATING_SPEED);
	// 		vehicleLat = (DoubleDataElement) stateData.get(DataElementKey.LATITUDE);
	// 		vehicleLon = (DoubleDataElement) stateData.get(DataElementKey.LONGITUDE);
	// 		IntersectionCollectionDataElement icde =
	// 				(IntersectionCollectionDataElement) stateData.get(DataElementKey.INTERSECTION_COLLECTION);
	// 		if (icde != null) {
	// 			inputIntersections = icde.value().intersections;
	// 		}

	// 		//check that all critical elements are present - will throw exception if missing
	// 		validateElement(curSpeedElement, "curSpeedElement", true);
	// 		validateElement(curAccelElement, "curAccelElement", true);
	// 		validateElement(operSpeedElem, "operSpeed", true);
	// 		validateElement(vehicleLat, "vehicleLat", true);
	// 		validateElement(vehicleLon, "vehicleLon", true);

	// 		// override the driver selected operating speed with one stored in the config file if it is valid (for testing)
	// 		operSpeed = operSpeedElem.value();
	// 		if (osOverride_ > 0.0) {
	// 			operSpeed = osOverride_ / Constants.MPS_TO_MPH;
	// 		}

	// 		//track how old the critical input elements are
	// 		long oldestTime;
	// 		if (respectTimeouts_) {
	// 			oldestTime = curSpeedElement.timeStamp();
	// 			if (curAccelElement.timeStamp() < oldestTime) oldestTime = curAccelElement.timeStamp();
	// 			if (vehicleLat.timeStamp() < oldestTime) oldestTime = vehicleLat.timeStamp();
	// 			if (vehicleLon.timeStamp() < oldestTime) oldestTime = vehicleLon.timeStamp();
	// 			if ((entryTime - oldestTime) > 0.9 * timeStepSize_) { //allow time for this method to execute within the time step
	// 				log_.warnf("TRAJ", "getSpeedCommand detects stale input data. curTime = %d, oldestTime = %d, timeStepSize_ = %d",
	// 						entryTime, oldestTime, timeStepSize_);
	// 				log_.warnf("TRAJ", "    speed is            %5d ms old", entryTime - curSpeedElement.timeStamp());
	// 				log_.warnf("TRAJ", "    accel is            %5d ms old", entryTime - curAccelElement.timeStamp());
	// 				log_.warnf("TRAJ", "    latitude is         %5d ms old", entryTime - vehicleLat.timeStamp());
	// 				log_.warnf("TRAJ", "    longitude is        %5d ms old", entryTime - vehicleLon.timeStamp());
	// 			}
	// 		}
	// 		curSpeed_ = curSpeedElement.value(); //this is the smoothed value
	// 		curAccel_ = curAccelElement.value(); //smoothed
	// 	}catch (Exception e) {
	// 		log_.error("TRAJ", "Unknown exception trapped in input processing: " + e.toString());
	// 		throw e;
	// 	}


	// 	////////// UNDERSTAND THE INTERSECTION GEOMETRY & WHERE WE FIT IN


	// 	double dtsb;
	// 	double lat = vehicleLat.value();
	// 	double lon = vehicleLon.value();
	// 	try {
	// 		//update internal record of intersections in view, sorted by distance from the vehicle (nearest first)
	// 		// ASSUMES all intersections within view are ones on our route (we will be traversing them)
	// 		Location vehicleLoc = new Location(lat, lon);
	// 		dtsb = intersectionManager_.updateIntersections(inputIntersections, vehicleLoc);

	// 		log_.debug("TRAJ", "getspeedCommand returned from call to updateIntersections");

	// 	}catch (Exception e) {
	// 		log_.error("TRAJ", "getSpeedCommand detected exception in intersection or spat calcs: " + e.toString());
	// 		throw e;
	// 	}


	// 	////////// RUN EAD TO GENERATE NEW PLAN

	// 	//take necessary actions based on the current state
	// 	double cmd = 0.0;  //the speed command we are to return;

	// 	//get speed command from EAD (assume it will handle position in stop box w/red light)
	// 	try {
	// 		log_.info("TrafficSignalPlugin", intersections_.toString());
	// 		cmd = ead_.getTargetSpeed(curSpeed_, operSpeed, curAccel_, intersections_);
	// 	}catch (Exception e) {
	// 		if (eadErrorCount_++ < MAX_EAD_ERROR_COUNT) {
	// 			cmd = prevCmd_;
	// 			log_.warnf("TRAJ", "Exception trapped from EAD algo: " + e.toString() +
	// 							"\n    Continuing to use previous command. " +
	// 							"curSpeed = %.2f, operSpeed = %.2f, dtsb = %.2f, timeRemaining = %.2f",
	// 					curSpeed_, operSpeed, dtsb, timeRemaining_);
	// 		}else {
	// 			log_.errorf("TRAJ", "Exception trapped by EAD algo: " + e.toString() +
	// 							"\n    Too many errors...rethrowing. " +
	// 							"curSpeed = %.2f, operSpeed = %.2f, dtsb = %.2f, timeRemaining = %.2f",
	// 					curSpeed_, operSpeed, dtsb, timeRemaining_);
	// 			throw e;
	// 		}
	// 	}

	// 	log_.debug("TRAJ", "getSpeedCommand completed executing current state.");

	// 	return null;
	// } //getSpeedCommand()

	public List<Node> plan(DataElementHolder stateData) throws Exception {
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
				inputIntersections = icde.value().intersections;
			}

			//check that all critical elements are present - will throw exception if missing
			validateElement(curSpeedElement, "curSpeedElement", true);
			validateElement(curAccelElement, "curAccelElement", true);
			validateElement(operSpeedElem, "operSpeed", true);
			validateElement(vehicleLat, "vehicleLat", true);
			validateElement(vehicleLon, "vehicleLon", true);

			// override the driver selected operating speed with one stored in the config file if it is valid (for testing)
			operSpeed = operSpeedElem.value();
			if (osOverride_ > 0.0) {
				operSpeed = osOverride_ / Constants.MPS_TO_MPH;
			}

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
			path = ead_.plan(curSpeed_, operSpeed, curAccel_, this.getSortedIntersections());
		}catch (Exception e) {
			// TODO find best place for if (eadErrorCount_++ < MAX_EAD_ERROR_COUNT) {
			// 	cmd = prevCmd_;
			// 	log_.warnf("TRAJ", "Exception trapped from EAD algo: " + e.toString() +
			// 					"\n    Continuing to use previous command. " +
			// 					"curSpeed = %.2f, operSpeed = %.2f, dtsb = %.2f, timeRemaining = %.2f",
			// 			curSpeed_, operSpeed, dtsb, timeRemaining_);
			// }else {
			// 	log_.errorf("TRAJ", "Exception trapped by EAD algo: " + e.toString() +
			// 					"\n    Too many errors...rethrowing. " +
			// 					"curSpeed = %.2f, operSpeed = %.2f, dtsb = %.2f, timeRemaining = %.2f",
			// 			curSpeed_, operSpeed, dtsb, timeRemaining_);
			// 	throw e;
			// }
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
		
		// double dtsb = Double.MAX_VALUE;
		// //if (intersections_ == null) {
		// //	log_.debug("TRAJ", "Entering updateIntersections. intersections_ object is null.");
		// //}else {
		// //	log_.debug("TRAJ", "Entering updateIntersections. intersections_ object is defined with size " + intersections_.size());
		// //}

		// // //loop through each previously known intersection
		// // if (intersections_.size() > 0) {
		// // 	for (IntersectionData i : intersections_) {
		// // 		//update the staleness counter
		// // 		++i.missingTimesteps;

		// // 		//clear out the old spat data (need to do this so none shows in case we missed one this time step)
		// // 		i.spat = null;
		// // 		i.currentPhase = NONE;
		// // 		i.timeToNextPhase = -1.0;

		// // 		//update the rough distance to it from the new vehicle position
		// // 		if (i.map != null) {
		// // 			Location loc = i.map.getRefPoint();
		// // 			i.roughDist = loc.distanceFrom(vehicleLoc);
		// // 		}
		// // 	}
		// // }
		// // intersectionsChanged_ = false;

		// if (inputIntersections != null  &&  inputIntersections.size() > 0) {
		// 	log_.debug("TRAJ", "updateIntersections has inputIntersections size = " + inputIntersections.size());

		// 	//loop through all input intersections
		// 	for (IntersectionData input : inputIntersections) {
		// 		int mapId = 0;
		// 		int spatId = 0;
		// 		int dist = Integer.MAX_VALUE; //cm

		// 		if (input.map != null) {
		// 			mapId = input.map.getIntersectionId();
		// 		}
		// 		if (input.spat != null) {
		// 			spatId = input.spat.getIntersectionId();
		// 		}
		// 		if (mapId > 0  &&  spatId > 0  &&  mapId != spatId) {
		// 			log_.warnf("TRAJ", "Input intersections with MAP ID = %d and SPAT ID = %d", mapId, spatId);
		// 			throw new Exception("Invalid intersections input with mismatched MAP/SPAT IDs.");
		// 		}
		// 		if (mapId == 0  &&  spatId == 0) {
		// 			log_.warn("TRAJ", "Input intersections with neither MAP nor SPAT attached.");
		// 			throw new Exception("Invalid intersections input with neither MAP nor SPAT attached.");
		// 		}
		// 		int id = Math.max(mapId, spatId); //since one of these might be zero

		// 		//if we don't want to deal with this intersection, then skip it
		// 		if (!wantThisIntersection(id)) {
		// 			continue;
		// 		}

		// 		//if we've already driven through this intersection, skip it
		// 		if (completedIntersections_.contains(id)) {
		// 			log_.debug("TRAJ", "updateIntersections - ignoring new data from id = " + id);
		// 			continue;
		// 		}

		// 		//calculate distance to this intersection's reference point
		// 		if (mapId > 0) {
		// 			Location loc = input.map.getRefPoint();
		// 			dist = loc.distanceFrom(vehicleLoc); //returns cm
		// 		}
		// 		log_.debug("TRAJ", "updateIntersections - preparing to look at known intersections for id = " + id);

		// 		//if the intersection is already known to us then update its info
		// 		int existingIndex = intersectionIndex(id);
		// 		if (existingIndex >= 0) {
		// 			IntersectionData known = intersections_.get(existingIndex);
		// 			known.missingTimesteps = 0;
		// 			if (mapId > 0) {
		// 				boolean firstMap = known.map == null;
		// 				if (firstMap  ||  known.map.getContentVersion() != input.map.getContentVersion()) {
		// 					known.map = input.map;
		// 					known.roughDist = dist;
		// 					intersectionsChanged_ = true;
		// 					log_.infof("TRAJ", "Added MAP data for intersection ID %d. Rough dist = %d cm",
		// 							input.map.getIntersectionId(), dist);
		// 				}

		// 				//if the previously known intersection has not yet had a MAP defined (it only had spats) then
		// 				// we need to re-sort the list by distance, since the rough distance to this was was
		// 				// previously unknown
		// 				if (firstMap  &&  intersections_.size() > 1) {
		// 					for (int k1 = 0;  k1 < intersections_.size() - 1;  ++k1) {
		// 						for (int k2 = k1 + 1;  k2 < intersections_.size();  ++k2) {
		// 							if (intersections_.get(k1).roughDist > intersections_.get(k2).roughDist) {
		// 								IntersectionData temp = intersections_.get(k1);
		// 								intersections_.set(k1, intersections_.get(k2));
		// 								intersections_.set(k2, temp);
		// 								intersectionsChanged_ = true;
		// 							}
		// 						}
		// 					}
		// 				}
		// 			}

		// 			//load the new spat data always, as the previous time step's data is now obsolete
		// 			// only load the raw spat message here, don't need to look at lane signal info
		// 			known.spat = input.spat;
		// 			known.currentPhase = input.currentPhase;
		// 			known.timeToNextPhase = input.timeToNextPhase;
		// 			known.timeToThirdPhase = input.timeToThirdPhase;

		// 		//else this is the first we've seen the intersection - add it to our list
		// 		}else {
		// 			intersectionsChanged_ = true;
		// 			input.roughDist = dist;
		// 			input.intersectionId = id;
		// 			input.missingTimesteps = 0;
		// 			log_.infof("TRAJ", "Adding new intersection ID %d to list, with rough distance = %d cm",
		// 					id, dist);

		// 			//insert it into the list in distance order, nearest to farthest
		// 			if (intersections_.size() > 0) {
		// 				boolean found = false;
		// 				for (int j = 0;  j < intersections_.size();  ++j) {
		// 					if (intersections_.get(j).roughDist > dist) {
		// 						intersections_.add(j, input);
		// 						found = true;
		// 						break;
		// 					}
		// 				}
		// 				if (!found) {
		// 					intersections_.add(input);
		// 				}
		// 			}else { //create first element in our known list
		// 				intersections_.add(input);
		// 			}
		// 		}
		// 	}

		// 	//at this point we are guaranteed to have at least one member of intersections_
		// 	//drop any intersections that we haven't seen in a long time
		// 	for (int i = intersections_.size() - 1;  i >= 0;  --i) { //count backwards to avoid index error when one is removed
		// 		IntersectionData inter = intersections_.get(i);
		// 		if (inter.missingTimesteps > maxSpatErrors_) {
		// 			log_.infof("TRAJ", "Removing intersection ID %d due to lack of signal for %d consecutive time steps.",
		// 					inter.intersectionId, maxSpatErrors_);
		// 			intersections_.remove(i);
		// 			intersectionsChanged_ = true;
		// 		}
		// 	}
		// }

		// //update the geometry for the nearest intersection and check our DTSB there
		// dtsb = updateNearestGeometry(vehicleLoc);
		// if (intersections_.size() > 0) {
		// 	intersections_.get(0).dtsb = dtsb;
		// }
		// double stopBoxWidth = intersectionGeom_ == null ? 0.0 : intersectionGeom_.stopBoxWidth();
		// ead_.setStopBoxWidth(stopBoxWidth);

		// //if we have transitioned to an egress lane or are no longer associated with a lane
		// // (should be somewhere near the center of the stop box) then
		// if (intersectionGeom_ != null  &&  spatReliableOnNearest_) {
		// 	int laneId = intersectionGeom_.laneId();
		// 	if ((laneId == -1 && prevApproachLaneId_ >= 0) ||
		// 			!intersectionGeom_.isApproach(laneId) || dtsb < -0.5 * stopBoxWidth) {
		// 		//remove the nearest intersection from the list, along with its associated map
		// 		// don't want to do this any sooner, cuz we may be stopped for red a little past the stop bar
		// 		log_.info("TRAJ", "updateIntersections removing current intersection. laneID = "
		// 					+ laneId + ", prevApproachLaneId = " + prevApproachLaneId_ + ", approach="
		// 					+ (intersectionGeom_.isApproach(laneId) ? "true" : "false") + ", dtsb = "
		// 					+ dtsb + ", stopBoxWidth = " + stopBoxWidth);
		// 		completedIntersections_.add(intersections_.get(0).intersectionId);
		// 		intersectionGeom_ = null;
		// 		map_ = null;
		// 		intersections_.remove(0);

		// 		//generate the geometry for the next current intersection
		// 		dtsb = updateNearestGeometry(vehicleLoc);
		// 		if (intersections_.size() > 0) {
		// 			intersections_.get(0).dtsb = dtsb;
		// 		}
		// 	}
		// }

		// return dtsb;
	}

	/**
	 * // TODO this is good logic to have but maybe not the place for it
	 * current speed/distance state is beyond the safe limit for approaching red or yellow signal : command maximum deceleration
	 * current speed/distance state is in the safe zone for the signal state : cmdIn
	 * 
	 * Note: jerk limit is ignored here, as this is an emergency maneuver. Also, emergency command is intentionally
	 * not dependent upon current actual speed, because measured speed may be drifting in wrong direction; we know
	 * we have to bring the vehicle to a stop along a sharp downward speed trajectory in a given distance, so 
	 * distance is all that's necessary to know.
	 * 
	 * Note (3/1/15):  At this moment the XGV/vehicle responsiveness is only something we know empirically. Available data 
	 * indicates that it will achieve serious acceleration only when abs(command - speed) > 1 m/s.  Further, it takes
	 * somewhere around 1 to 1.5 sec to achieve a significant amount of torque (positive or negative) that begins to
	 * accelerate the vehicle in the desired way, even with an instantaneous 1 m/s command premium over actual speed.
	 */
	private double applyFailSafeCheck(double cmdIn, double distance, double speed) {
		return cmdIn;
		// double cmd = cmdIn;
		
		// //if the failsafe toggle has been turned off or if we are not approaching an intersections then return
		// if (!allowFailSafe_  ||  phase_ == NONE) {
		// 	return cmd;
		// }
		
		// //set the acceleration limit higher than in normal ops since this is for handling emergency situations (this will be a positive number)
		// double decel = failSafeDecelFactor_*accelMgr_.getAccelLimit();
		
		// //compute the distance that will be covered during vehicle response lag
		// double lagDistance = failSafeResponseLag_ * speed;
		
		// //determine emergency stop distance for our current speed (limited to be non-negative)
		// double emerDistance = Math.max((0.5*speed*speed / decel + lagDistance + failSafeDistBuf_), 0.0);
		
		// //are we in fail-safe mode?
		// if (failSafe(emerDistance, distance, speed)) {

		// 	//if we haven't yet reached the bar, we have time to slow more gradually
		// 	double failsafeCmd;
		// 	if (distance > 0.0) {
		// 		//determine where we will be one time step in the future, going at the current speed (may be negative!)
		// 		// plan to stop failSafeDistBuf_ short of the stop bar, and account for the response lag time
		// 		double futureDistance = distance - 0.001*timeStepSize_*speed - lagDistance - failSafeDistBuf_;
		// 		//determine the speed we want to have at that point in time to achieve a smooth slow-down
		// 		double desiredSpeed = Math.sqrt(Math.max(2.0*decel*futureDistance, 0.0));

		// 		//determine control adjustment that provides a command sufficiently below the actual speed that the controller will take it seriously
		// 		double adj = calcControlAdjustment(speed, desiredSpeed, -decel);
		// 		double rawCmd = desiredSpeed + adj;

		// 		//compute the new fail-safe command
		// 		failsafeCmd = Math.max(Math.min(rawCmd, speed), 0.0);

		// 		//under no circumstances do we want the resultant command to be larger than the previous one!
		// 		if (failsafeCmd > prevCmd_) {
		// 			failsafeCmd = 0.99*prevCmd_;
		// 		}
		// 		log_.debugf("TRAJ", "Fail-safe futureDistance = %.2f, speed = %.2f, desiredSpeed = %.2f, rawCmd = %.2f, adj = %.2f, failsafeCmd = %.2f",
		// 				futureDistance, speed, desiredSpeed, rawCmd, adj, failsafeCmd);
		// 	}else {
		// 		//need immediate stop!
		// 		failsafeCmd = 0.0;
		// 		log_.debugf("TRAJ", "Fail-safe commanding 0 since we are past the stop bar!");
		// 	}

		// 	//if it is less than the input command then use it
		// 	if (failsafeCmd < cmdIn) {
		// 		cmd = failsafeCmd;
		// 		log_.warn("TRAJ", "FAIL-SAFE has overridden the calculated command");
		// 	}

		// }
		
		// return cmd;
	}

	/**
	 * !failSafeMode_  &&  distance < emerDist  &&  can't get through a green light at current speed : true, and set failSafeMode_
	 * failSafeMode_  &&  speed == 0 for 5 consecutive time steps : false, and clear failSafeMode_
	 * otherwise, return current failSafeMode_ without changing it
	 * 
	 * The idea is that once we decide we need a failsafe (emergency) stop, then we are committed to it until the deed is done.
	 */
	private boolean failSafe(double emerDist, double distance, double speed) {
		
		//if already in failsafe mode then
		if (failSafeMode_) {
			//if speed has been zero for enough time steps then
			if (speed < 0.001  &&  ++numStepsAtZero_ > STOP_DAMPING_TIMESTEPS) {
				//turn off failsafe and trust the EAD to take over the departure
				failSafeMode_ = false;
				numStepsAtZero_ = 0;
				log_.info("TRAJ", "///// Fail-safe has been turned off.");
			}
		//else
		}else {
			//if current DTSB < our emergency stop distance for this speed and speed is non-zero then
			// (the only way this will be triggered if distance < 0 is if we are creeping past the stop bar,
			// so no need to guard against negative distance since it will be real close to zero; we're not
			// trying to escape through the beginning of yellow or red)
			if (distance < emerDist  &&  speed > 0.08) {
		
				//determine time to cross stop bar at current speed
				double timeToCross = Math.max(distance/speed, 0.0);
			
				//determine if we have to stop (go into failsafe mode): if
				//	green and we can't make it through before expiring, or
				//	yellow, or
				//	red and it will still be red when we arrive
				failSafeMode_ = (phase_.equals(SignalPhase.GREEN)  &&  timeToCross >= timeRemaining_)  ||
								 phase_.equals(SignalPhase.YELLOW)                                     ||
								(phase_.equals(SignalPhase.RED)    &&  timeToCross <= timeRemaining_);
				if (failSafeMode_) {
					log_.warnf("TRAJ", "///// FAIL-SAFE ACTIVATED; will remain active until vehicle stops. timeToCross = %.2f", timeToCross);
				}
			}
		}

		return failSafeMode_;
	}
	
	/**
	 * always : adjustment to be added to the desired speed to get the final command
	 */
	private double calcControlAdjustment(double actSpeed, double desiredSpeed, double accel) {
		double p = cmdAccelGain_*(accel - curAccel_) + cmdSpeedGain_*(desiredSpeed - actSpeed) - cmdBias_; //note subtracting bias since we know we're slowing
		if (p > maxCmdAdj_) {
			p = maxCmdAdj_;
		}else if (p < -maxCmdAdj_) {
			p = -maxCmdAdj_;
		}
		
		return p;
	}
	
	private double				osOverride_;		//FOR DEBUGGING ONLY - allows the config file to override the driver selected operating speed, m/s
	private long				timeStepSize_;		//duration of a single time step, ms
	private boolean				stopConfirmed_;		//are we at a complete stop?
	private boolean				respectTimeouts_;	//should we honor the established timeouts?
	private int[]				intersectionIds_;	//array of IDs of intersections that we will pay attention to
	private int					maxSpatErrors_;		//max allowable number of consecutive spat data errors
	private double				speedLimit_;		//upper limit allowed for speed, m/s
	private double				maxJerk_;			//max allowed jerk, m/s^3
	private double				curSpeed_;			//current vehicle speed, m/s
	private double				curAccel_;			//current acceleration (smoothed), m/s^2
	private SignalPhase			phase_;				//the signal's current phase
	private double				timeRemaining_;		//time remaining in the current signal phase, sec
	private IEad				ead_;				//the EAD model that computes the speed commands
	private boolean				accelLimiter_;		//is acceleration limiter used?
	private boolean				jerkLimiter_;		//is jerk limiter used?
	private double				maxCmdAdj_;			//difference (m/s) needed between current speed and command given to the XGV to get it to respond quickly
	private boolean				allowFailSafe_;		//does the user want failsafe logic involved?
	private boolean				failSafeMode_;		//are we in failsafe mode (overriding all other trajectory calculations)?
	private double				failSafeDistBuf_;	//distance buffer that failsafe will subtract from stop bar location, meters
	private double				failSafeResponseLag_; //time that failsafe logic allows for the vehicle to respond to a command change, sec
	private	int					numStepsAtZero_;	//number of consecutive time steps with speed of zero (after first motion) for failsafe use
	private double				cmdSpeedGain_;		//gain applied to speed difference for fail-safe calculations
	private double				cmdAccelGain_;		//gain applied to acceleration for fail-safe calculations
	private double				cmdBias_;			//bias applied to command premium for fail-safe calculations

	private static final int 	STOP_DAMPING_TIMESTEPS = 19; //num timesteps before stopping vibrations disappear
	private static ILogger		log_ = LoggerManager.getLogger(Trajectory.class);

	private EadIntersectionManager intersectionManager_ = new EadIntersectionManager();
}
