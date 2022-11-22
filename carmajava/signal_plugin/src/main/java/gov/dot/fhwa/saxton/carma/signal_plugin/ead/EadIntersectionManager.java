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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementHolder;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DataElementKey;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.DoubleDataElement;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.PhaseDataElement;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Class responsible for updating intersection data used for identifying the current intersection and lane of the host vehicle.
 */
public class EadIntersectionManager {
	/**
	 * Constructor
	 * @throws Exception
	 */
	public EadIntersectionManager() throws Exception {
		//Get application context
		IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
		
		if (config == null) {
			throw new IllegalArgumentException("Null config returned by GlidepathApplicationContext");
		}
		
		// Load constants for intersection geometry processing
		cteThreshold_ = config.getIntValue("ead.cte.threshold");
		periodicDelay_ = config.getIntValue("periodicDelay");
		usableLanes_ = config.getIntegerList("usableLaneIds");
		
		for (Integer i: usableLanes_)
		{
			log_.warn("TRAJI", "Usable %d", i);
		}

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

		//initialize other members
		sortedIntersections_.set(new PriorityQueue<IntersectionData>(
			(i1, i2) -> {return i1.getRoughDist() - i2.getRoughDist();}));
		completedIntersections_ = new HashSet<>();
		prevApproachLaneId_.set(-1);
	}

	/**
	 * Returns a list of known intersections sorted from nearest to farthest
	 * 
	 * @return A list of IntersectionData sorted from nearest to farthest
	 */
	public Queue<IntersectionData> getSortedIntersections() {
		return sortedIntersections_.get();
	}

	private boolean validIntersection(IntersectionData input) {

		if (input.getMap() == null || input.getSpat() == null) {
			log_.warn("TRAJ", "Input intersections had null map or spat");
			return false;
		}

		int	mapId = input.getMap().getIntersectionId();
		int spatId = input.getSpat().getIntersectionId();

		if (mapId != spatId) {
			log_.warnf("TRAJ", "Input intersections with MAP ID = %d and SPAT ID = %d", mapId, spatId);
			return false;
		}
		// TODO map and spat ids of 0 are valid and should be accounted for
		if (mapId == 0  ||  spatId == 0) {
			log_.warn("TRAJ", "Input intersections with neither MAP nor SPAT attached.");
			return false;
		}

		//if we don't want to deal with this intersection, then skip it
		if (!wantThisIntersection(mapId)) {
			log_.debug("TRAJ", "Intersection ignored due to id not being in desired id list. Id: " + mapId);
			return false;
		}

		//if we've already driven through this intersection, skip it
		if (completedIntersections_.contains(mapId)) {
			log_.debug("TRAJ", "updateIntersections - ignoring new data from id = " + mapId);
			return false;
		}

		return true;
	}

	/**
	 * Refreshes the internal list of intersections in view with the latest input data.
	 *
	 * @param inputIntersections - list of intersections sensed by the vehicle in this time step
	 * @param vehicleLoc - current location of the vehicle
	 * 
	 * @return distance to stop bar of the nearest intersection
	 */
	public double updateIntersections(List<IntersectionData> inputIntersections, Location vehicleLoc) throws Exception {
		double dtsb = Double.MAX_VALUE;

		Queue<IntersectionData> sortedIntersections = new PriorityQueue<IntersectionData>(
				(i1, i2) -> {return i1.getRoughDist() - i2.getRoughDist();});
		
		if (inputIntersections == null  ||  inputIntersections.size() <= 0) {
			sortedIntersections_.set(sortedIntersections); // Store sorted intersections
			throw new IllegalArgumentException("updateIntersections called with no intersection data");
		}

		log_.debug("TRAJ", "updateIntersections has inputIntersections size = " + inputIntersections.size());

		//loop through all input intersections and only keep those whose maps we are on
		for (IntersectionData input : inputIntersections) {

			if (!validIntersection(input)) { // Check that intersection is valid
				continue;
			}

			//calculate distance to this intersection's reference point
			Location loc = input.getMap().getRefPoint();
			input.setRoughDist(loc.distanceFrom(vehicleLoc)); //returns cm

			IntersectionGeometry geometry = computeIntersectionGeometry(vehicleLoc, input.getMap());
			if (geometry != null) {
				input.setDtsb(geometry.dtsb());
				input.setLaneId(geometry.laneId());
				input.setGeometry(geometry);
				input.setStopBoxWidth(geometry.stopBoxWidth());

				DoubleDataElement dde = (DoubleDataElement) input.getSpat().getSpatForLane(input.getLaneId()).get(DataElementKey.SIGNAL_TIME_TO_NEXT_PHASE);
				if (dde != null) {
						log_.debug("INTR","Updated timeToNextPhase: " + dde.value());
						input.setTimeToNextPhase(dde.value());
				}

				PhaseDataElement pde = (PhaseDataElement) input.getSpat().getSpatForLane(input.getLaneId()).get(DataElementKey.SIGNAL_PHASE);
				if (pde != null) {
						log_.debug("INTR","Updated phase: " + pde.value());
						input.setCurrentPhase(pde.value());
				}

				sortedIntersections.add(input); // Add the intersection to our sorted set
			}

			log_.debug("TRAJ", "updateIntersections - preparing to look at known intersections for id = " + input.getIntersectionId());
		}

		if (sortedIntersections.size() <= 0) {
			log_.debug("TRAJ", "No valid intersections detected");
			sortedIntersections_.set(sortedIntersections); // Store sorted intersections
			return dtsb;
		}

		// Identify current intersection
		IntersectionGeometry nearGeometry = sortedIntersections.peek().getGeometry();
		dtsb = sortedIntersections.peek().getDtsb();

		//if we have transitioned to an egress lane or are no longer associated with a lane
		// (should be somewhere near the center of the stop box) then
		if (nearGeometry != null) { 
			int laneId = nearGeometry.laneId();
			// If the vehicle is not on a lane (possibly in the stop box (laneId == -1)) AND we were previously on an approach lane
			// OR The current lane is an actual lane but not an approach lane
			// OR The current dtsb is negative and less than half the stop box width (just past the stop bar)
			// Then we have passed the stop bar of this intersection
			double stopBoxWidth = nearGeometry.stopBoxWidth();
			if ((laneId == -1 && prevApproachLaneId_.get() >= 0) ||
					(laneId != -1 && !nearGeometry.isApproach(laneId)) || dtsb < -0.5 * stopBoxWidth) {
				//remove the nearest intersection from the list, along with its associated map
				// don't want to do this any sooner, cuz we may be stopped for red a little past the stop bar
				log_.info("TRAJ", "updateIntersections removing current intersection. laneID = "
							+ laneId + ", prevApproachLaneId = " + prevApproachLaneId_ + ", approach="
							+ (nearGeometry.isApproach(laneId) ? "true" : "false") + ", dtsb = "
							+ dtsb + ", stopBoxWidth = " + stopBoxWidth);
				completedIntersections_.add(sortedIntersections.poll().getIntersectionId());
				nearGeometry = null;
				dtsb = Double.MAX_VALUE;

				//generate the geometry for the next current intersection
				if (sortedIntersections.size() > 0) {
					dtsb = sortedIntersections.peek().getDtsb();
				}
			}
			
			if (nearGeometry != null && nearGeometry.isApproach(laneId)) {
				prevApproachLaneId_.set(laneId);
				sortedIntersections.peek().setLaneId(laneId);
				log_.debug("INTR", "Trying to update spat for lane: " + laneId);
			}
		}


		sortedIntersections_.set(sortedIntersections); // Store sorted intersections

		return dtsb;
	}

	/**
	 * Function generates a new IntersectionGeometry object using the provided vehicle location and map message
	 * 
	 * @param vehicleLoc The vehicle location as a gps fix
	 * @param mapMessage The map message to generate geometry for
	 * 
	 * @return An initialized IntersectionGeometry object if the vehicle location could be matched with a lane. Null otherwise
	 */
	private IntersectionGeometry computeIntersectionGeometry(Location vehicleLoc, MapMessage mapMessage) {
		//find the distance to the stop bar of the approach to the current intersection (negative values mean we
		// are crossing the box and about to depart the intersection)
		double dtsb = Double.MAX_VALUE; // TODO change this to Integer.MAX_VALUE all over
		log_.debug("TRAJ", "computeIntersectionGeometry entered with intersectionId: " + mapMessage.getIntersectionId());
		
		IntersectionGeometry geometry = new IntersectionGeometry(cteThreshold_, periodicDelay_);
		geometry.initialize(mapMessage);

		//compute the current vehicle geometry relative to the intersections
		boolean associatedWithLane = geometry.computeGeometry(vehicleLoc.lat(), vehicleLoc.lon());
		//get the DTSB
		dtsb = geometry.dtsb();

		//if the computation was successful (vehicle can be associated with a lane) then
		if (associatedWithLane) {
			int laneId = geometry.laneId();

			if (usableLanes_.isEmpty() || ( usableLanes_.contains(laneId) && dtsb < (Integer.MAX_VALUE - 5.0)) )
			{
				int idx = geometry.getLaneIndex();
				log_.debugf("TRAJ",
					"  computeIntersectionGeometry: we are %.1f m from stop bar on a usable lane %d of intersection %d",
					dtsb, laneId, mapMessage.getIntersectionId());
				currentLane_ = laneId;
				currentLaneIdx_ = idx;
				
			} else
			{
				log_.debugf("TRAJ",
					"  computeIntersectionGeometry: we are %.1f m from stop bar on lane %d of intersection %d",
					dtsb, laneId, mapMessage.getIntersectionId());

				if (currentLaneIdx_ != -1 && dtsb < (Integer.MAX_VALUE - 5.0))
				{
					geometry.setLaneIndex(currentLaneIdx_);
				}
			}
			
			
		} else {
			return null;
		}

		return geometry;
	}
	
	private boolean wantThisIntersection(int thisId) {
		boolean wanted = false;
		log_.debug("TRAJ", "Checking if intersection is needed id: " + thisId);
		for (int id : intersectionIds_) {
			if (thisId == id) {
				wanted = true;
				break;
			}
		}

		log_.debug("TRAJ", "Intersection " + thisId + " wanted?: " + wanted);
		return wanted;
	}
	
	private AtomicReference<Queue<IntersectionData>> sortedIntersections_ = new AtomicReference<>(); //all of the viable intersections currently in view
	private HashSet<Integer>	completedIntersections_; //IDs of intersections we have already passed through
	private AtomicReference<Integer> prevApproachLaneId_ = new AtomicReference<>();;//value of approachLaneId from the previous time step
	private int[]				intersectionIds_;	//array of IDs of intersections that we will pay attention to
	private static ILogger		log_ = LoggerManager.getLogger(EadIntersectionManager.class);
	private final int cteThreshold_;
	private final int periodicDelay_;
	private final List<Integer> usableLanes_;
	private int currentLane_ = -1;
	private int currentLaneIdx_ = -1;
}
