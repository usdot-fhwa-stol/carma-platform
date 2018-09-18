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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IntersectionData;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.io.IOException;
import java.util.*;

/**
 * TODO. Handles intersection conversion and dtsb calc
 */
public class EadIntersectionManager {
	/**
	 * Constructor
	 * @throws Exception
	 */
	public EadIntersectionManager() throws Exception {
		//Get application context
		IGlidepathAppConfig config = GlidepathApplicationContext.getInstance().getAppConfig();
		assert(config != null);

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
		sortedIntersections_ = new PriorityQueue<>(
			(IntersectionData i1, IntersectionData i2) -> {
				return i1.roughDist - i2.roughDist;
			});
		completedIntersections_ = new HashSet<>();
		prevApproachLaneId_ = -1;
		intersectionGeom_ = null;
		map_ = null;
	}

	/**
	 * Returns a list of known intersections sorted from nearest to farthest
	 * 
	 * @return A list of IntersectionData sorted from nearest to farthest
	 */
	public Queue<IntersectionData> getSortedIntersections() {
		return sortedIntersections_;
	}

	/**
	 * Refreshes the internal list of intersections in view with the latest input data.
	 * TODO
	 *
	 * @param inputIntersections - list of intersections sensed by the vehicle in this time step
	 * @param vehicleLoc - current location of the vehicle
	 * @return distance to stop bar of the nearest intersection
	 */
	public double updateIntersections(List<IntersectionData> inputIntersections, Location vehicleLoc) throws Exception {
		double dtsb = Double.MAX_VALUE;

		Queue<IntersectionData> sortedIntersections = new PriorityQueue<>(
		(IntersectionData i1, IntersectionData i2) -> {
			return i1.roughDist - i2.roughDist;
		});
		
		if (inputIntersections == null  ||  inputIntersections.size() <= 0) {
			throw new IllegalArgumentException("updateIntersections called with no intersection data");
		}

		log_.debug("TRAJ", "updateIntersections has inputIntersections size = " + inputIntersections.size());

		//loop through all input intersections
		for (IntersectionData input : inputIntersections) {

			if (!validIntersection(input)) { // Check that intersection is valid
				continue;
			}

			//calculate distance to this intersection's reference point
			Location loc = input.map.getRefPoint();
			input.roughDist = loc.distanceFrom(vehicleLoc); //returns cm

			sortedIntersections.add(input); // Add the intersection to our sorted set

			log_.debug("TRAJ", "updateIntersections - preparing to look at known intersections for id = " + input.intersectionId);
		}

		//update the geometry for the nearest intersection and check our DTSB there
		dtsb = updateNearestGeometry(vehicleLoc, sortedIntersections);
		if (sortedIntersections.size() > 0) {
			sortedIntersections.peek().dtsb = dtsb;
		}
		double stopBoxWidth = intersectionGeom_ == null ? 0.0 : intersectionGeom_.stopBoxWidth(); // TODO we need ead to read the stop box width from the intersection data automatically

		//if we have transitioned to an egress lane or are no longer associated with a lane
		// (should be somewhere near the center of the stop box) then
		if (intersectionGeom_ != null) {
			int laneId = intersectionGeom_.laneId();
			// If the vehicle is in the stop box (laneId == -1) AND we were previously on an approach lane
			// OR The current lane is not an approach lane
			// OR The current dtsb is negative and less than half the stop box width (just past the stop bar)
			// Then we have passed the stop bar of this intersection
			if ((laneId == -1 && prevApproachLaneId_ >= 0) ||
					!intersectionGeom_.isApproach(laneId) || dtsb < -0.5 * stopBoxWidth) {
				//remove the nearest intersection from the list, along with its associated map
				// don't want to do this any sooner, cuz we may be stopped for red a little past the stop bar
				log_.info("TRAJ", "updateIntersections removing current intersection. laneID = "
							+ laneId + ", prevApproachLaneId = " + prevApproachLaneId_ + ", approach="
							+ (intersectionGeom_.isApproach(laneId) ? "true" : "false") + ", dtsb = "
							+ dtsb + ", stopBoxWidth = " + stopBoxWidth);
				completedIntersections_.add(sortedIntersections.poll().intersectionId);
				intersectionGeom_ = null;
				map_ = null;

				//generate the geometry for the next current intersection
				dtsb = updateNearestGeometry(vehicleLoc, sortedIntersections);
				if (sortedIntersections.size() > 0) {
					sortedIntersections.peek().dtsb = dtsb;
				}
			}

			if (intersectionGeom_.isApproach(laneId)) {
				prevApproachLaneId_ = laneId;
			}
		}

		sortedIntersections_ = sortedIntersections;

		return dtsb;
	}

	private boolean validIntersection(IntersectionData input) {

		if (input.map == null || input.spat == null) {
			log_.warn("TRAJ", "Input intersections had null map or spat");
			return false;
		}

		int	mapId = input.map.getIntersectionId();
		int spatId = input.spat.getIntersectionId();

		if (mapId != spatId) {
			log_.warnf("TRAJ", "Input intersections with MAP ID = %d and SPAT ID = %d", mapId, spatId);
			return false;
		}
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
	 * Updates geometry for the nearest intersection and computes the DTSB relative to it if vehicle is on a
	 * known approach lane.  If it is not on an intersection map then it cannot be associated with any lane.
	 *
	 * @param vehicleLoc - current vehicle location
	 * @return DTSB relative to nearest intersection on the designated approach lane; very large if no association
	 */
	private double updateNearestGeometry(Location vehicleLoc, Queue<IntersectionData> sortedIntersections) {
		//find the distance to the stop bar of the approach to the current intersection (negative values mean we
		// are crossing the box and about to depart the intersection)
		double dtsb = Double.MAX_VALUE;
		log_.debug("TRAJ", "updateNearestGeometry entered with " + sortedIntersections.size() + " intersections.");
		if (sortedIntersections.size() > 0) {
			boolean validMap = loadNewMap(sortedIntersections.peek().map); //creates intersectionGeom_ if one is valid
			log_.debug("TRAJ", "updateNearestGeometry: validMap = " + validMap + " and intersectionGeom "
						+ (intersectionGeom_ == null ? "is" : "is not") + " null.");

			if (validMap) {
				//compute the current vehicle geometry relative to the intersections
				boolean associatedWithLane = intersectionGeom_.computeGeometry(vehicleLoc.lat(), vehicleLoc.lon());
				//get the DTSB
				dtsb = intersectionGeom_.dtsb();
				//log_.debug("TRAJ", "updateNearestGeometry: dtsb = " + dtsb);

				//if the computation was successful (vehicle can be associated with a lane) then
				if (associatedWithLane) {
					int laneId = intersectionGeom_.laneId();
					log_.debugf("TRAJ",
							"  updateNearestGeometry: we are %.1f m from stop bar on lane %d of intersection %d",
							dtsb, laneId, map_.getIntersectionId());
				}
			}
		}

		return dtsb;
	}

	/**
	 * newMap exists and is different from previously stored map : initialize intersections geometry model from it
	 * else : do nothing
	 */
	private boolean loadNewMap(MapMessage newMap) {
		log_.debug("TRAJ", "loadNewMap entered. newMap = " + (newMap == null ? "null" : "valid"));
		//if there is a new incoming MAP message and it belongs to the list of intersections we care about then
		if (newMap != null  &&  wantThisIntersection(newMap.getIntersectionId())) {

			//if it is different from the MAP we were working with in the previous time step then
			if (map_ == null  ||  newMap.getIntersectionId() != map_.getIntersectionId()  ||
					newMap.getContentVersion() != map_.getContentVersion()) {
		
				//create a new intersection object
				map_ = newMap;
				intersectionGeom_ = new IntersectionGeometry();
				intersectionGeom_.initialize(map_);
			}
		}

		return map_ != null; //if we have handled one at any time in the past we are now good
	}
	
	
	private boolean wantThisIntersection(int thisId) {
		boolean wanted = false;
		log_.info("TRAJ", "Checking if intersection is needed id: " + thisId);
		for (int id : intersectionIds_) {
			if (thisId == id) {
				wanted = true;
				break;
			}
		}

		return wanted;
	}
	
	private Queue<IntersectionData> sortedIntersections_; //all of the viable intersections currently in view
	private HashSet<Integer>	completedIntersections_; //IDs of intersections we have already passed through
	private int					prevApproachLaneId_;//value of approachLaneId from the previous time step
	private int[]				intersectionIds_;	//array of IDs of intersections that we will pay attention to
	private IntersectionGeometry intersectionGeom_;	//vehicle's relationship to the nearest intersection, if on its map
	private MapMessage			map_;				//the MAP message that describes the nearest intersection (we may not be on it)
	private static ILogger		log_ = LoggerManager.getLogger(EadIntersectionManager.class);
}
