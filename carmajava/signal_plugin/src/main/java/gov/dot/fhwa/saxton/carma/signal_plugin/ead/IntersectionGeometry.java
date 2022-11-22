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

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Lane;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.map.MapMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.Vector;

// This class represents the geometry of a single road intersections and the position of the vehicle relative to it.  It is
// constructed from a MAP message.  Given a vehicle position, it will attempt to determine which lane the vehicle is driving on
// (aka the associated lane).  There are situations where such an association may be impossible, such as the indicated
// vehicle position is nowhere near any of the defined lanes.

public class IntersectionGeometry {

	/**
	 * Constructs the IntersectionGeometry object
	 * 
	 * @param cteThreshold The threshold in cm over which the vehicle will no longer be associated with a lane
	 * @param periodicDelay The expected data rate of position updates
	 */
	public IntersectionGeometry(int cteThreshold, int periodicDelay) {
		cteThreshold_ = cteThreshold;
		timeStep_ = 0.001*(double)periodicDelay;
		largeMovement_ = (int)(70.0/Constants.MPS_TO_MPH * 100.0 * timeStep_ * 2.5); //70 MPH, 100 cm/m, time step, safety factor
		laneIndex_ = -1;
		laneDtsb_ = Integer.MAX_VALUE;
		cte_ = Integer.MAX_VALUE;
		prevLaneApproach_ = false;
		prevLaneIndex_ = -1;
		prevLaneIndexInBox_ = -1;
		prevDtsb_ = Integer.MAX_VALUE;
		lastApproachDtsb_ = 0;
		firstEgressDtsb_ = 0;
		map_ = null;
		laneGeo_ = null;
		stopBoxWidth_ = 0.0;
	}
	
	/**
	 * always : parses incoming mapMsg and creates the intersections geometry, including defining the
	 * 			stop box width
	 *
	 * @param mapMsg - a valid MAP message retrieved from the ASD device that describes the intersections
	 */
	public void initialize(MapMessage mapMsg) {
		
		long startTime = System.currentTimeMillis();

		//store the MAP message
		map_ = mapMsg;
		
		//set up the array for each lane
		int numLanes = mapMsg.numLanes();
		if (laneGeo_ == null) {
			laneGeo_ = new LaneGeometry[numLanes];
			for (int j = 0;  j < numLanes;  ++j) {
				laneGeo_[j] = null;
			}
		}
			
		//loop through each lane in the MAP message
		try {
			for (int i = 0;  i < numLanes;  ++i) {
				//if a geometry hasn't already been constructed for this lane then
				if (laneGeo_[i] == null) {
					//create the lane geometry from the current lane in the MAP message
					laneGeo_[i] = new LaneGeometry(mapMsg.getRefPoint(), mapMsg.getLane(i));
					//log_.debugf("INTR", "Geometry initialized for lane index %d", i);
				}	
			} //end loop on lanes
		}catch (Exception e) {
			log_.warn("INTR", "Exception trapped defining lane geometry: " + e.getMessage());
		}
		
		//find the largest distance between any pair of stop bars (node 0 of each lane)
		for (int i = 0;  i < numLanes - 1;  ++i) {
			CartesianPoint2D sb1 = laneGeo_[i].getStopBar();
			
			for (int j = i + 1;  j < numLanes;  ++j) {
			
				CartesianPoint2D sb2 = laneGeo_[j].getStopBar();
				double dist = 0.01*sb1.distanceFrom(sb2);
				
				if (dist > stopBoxWidth_) {
					stopBoxWidth_ = dist;
				}
			}
		}
		
		log_.debugf("INTR", "End of initialize for intersections ID %d, took = %d ms, stopBoxWidth = %.2f",
				map_.getIntersectionId(), System.currentTimeMillis()-startTime, stopBoxWidth_);
	}
	
	/**
	 * intersections initialized : determine which lane(s) the vehicle is associated with and its position relative to that lane
	 * intersections not initialized : false
	 * 
	 * @param vehicleLat in degrees
	 * @param vehicleLon in degrees
	 * @return true if vehicle is successfully associated with a single lane; false if not
	 */
	public boolean computeGeometry(double vehicleLat, double vehicleLon) {
		boolean success = false;
		
		Location vehicle = new Location(vehicleLat, vehicleLon);
		class Candidate {
			private int		index;
			private boolean	approach;
			private int		dtsb; //this is simply dist to the stop bar of THIS lane, not necessarily of the lane we approached the intersections on
			private int		cte;
		}
		Vector<Candidate> candidateLanes = new Vector<Candidate>();
		
		Candidate prevLane = new Candidate();
		prevLane.index = prevLaneIndex_;
		prevLane.approach = prevLaneApproach_;

		Candidate chosenLane = null;

		// ----- initial screening for candidate lanes that we might be close to - see if we're in any bounding boxes
		//       Caution: if the bounding box thresholds are too small relative to the size of the intersections, there
		//       may be a no-man's land near the center of the stop box that is not in any of the lanes' bounding boxes!
		//       This would fail the first screen and no DTSB would be calculated.
		
		//loop on all lanes
		for (int i = 0;  i < map_.numLanes();  ++i) {
			//if the vehicle is in this lane's bounding box then
			if (laneGeo_[i].inBoundingBox(vehicle)) {
				
				//get DTSB & CTE relative to this lane
				Candidate cand = new Candidate();
				cand.dtsb = laneGeo_[i].dtsb(vehicle);
				cand.cte = laneGeo_[i].cte(vehicle);
				
				//record the lane index & type as a candidate
				cand.index = i;
				cand.approach = map_.getLane(i).isApproach();
				candidateLanes.add(cand);
			}
		}
		log_.debugf("INTR", "computeGeometry: initial screening found %d candidate lanes", candidateLanes.size());
		
		// ----- for all candidates found above let's spend the time to see if our cross-track error (lateral distance to lane centerline)
		// ----- is small enough to call this "our" lane in a variety of possible situations
		
		//if at least one candidate lane was identified then
		if (candidateLanes.size() > 0) {

			//if only one candidate lane was identified then
			if (candidateLanes.size() == 1) {
				//if it is the same lane as used previously or (this lane is egress and previous was approach) then
				if (candidateLanes.get(0).index == prevLane.index  ||  
						(!candidateLanes.get(0).approach  &&  prevLane.approach)) {
					//store it and indicate success
					chosenLane = candidateLanes.get(0);
					success = true;
				//else
				}else {

					//if its CTE is within the acceptable threshold then
					if (candidateLanes.get(0).cte < cteThreshold_) {
						//if previous DTSB is significantly negative (we are well into the box) then
						if (prevDtsb_ < -Constants.THRESHOLD_DIST) {
							//store the index of the approach lane that got us here in case we don't associate with another lane
							prevLaneIndexInBox_ = Math.max(prevLaneIndex_, prevLaneIndexInBox_); //one of these will be -1
							//if candidate is an egress then
							if (!candidateLanes.get(0).approach) {
								//select it
								chosenLane = candidateLanes.get(0);
								success = true;
							}

						//else - take it
						}else {
							chosenLane = candidateLanes.get(0);
							success = true;
						}
					}
				}
		
			//else (multiple candidates were identified)
			}else {
				//if vehicle is approaching the intersections then
				if (prevDtsb_ >= 0) {
					//if we are close to the stop bar then
					if (prevDtsb_ < Constants.THRESHOLD_DIST  &&  prevLane.index >= 0) {
						//use the same lane as previous time step (too many lanes near the stop box to try sorting them out)
						chosenLane = prevLane;
						chosenLane.cte = laneGeo_[chosenLane.index].cte(vehicle);
						chosenLane.dtsb = laneGeo_[chosenLane.index].dtsb(vehicle);
					//else
					}else {
						//choose the approach lane with the smallest CTE
						int smallestCte = Integer.MAX_VALUE;
						for (int j = 0;  j < candidateLanes.size();  ++j) {
							Candidate c = candidateLanes.get(j);
							if (c.approach  &&  c.cte < smallestCte) {
								chosenLane = c;
								smallestCte = c.cte;
							}
						}
					}
					/*****
					if (chosenLane == null) {
						log_.debugf("INTR", "approaching: prevDtsb = %d, no lane chosen.", prevDtsb_);
					}else {
						log_.debugf("INTR", "approaching: prevDtsb = %d, chose lane index %d", prevDtsb_, chosenLane.index);
					}
					*****/
		
				//else (beyond stop bar - in intersections or we are already departing)
				}else {
					//if we are sufficiently downtrack of the stop bar (same as uptrack closeness threshold) then
					if (prevDtsb_ < -Constants.THRESHOLD_DIST) {
						//store the index of the approach lane that got us here in case we don't associate with another lane
						prevLaneIndexInBox_ = Math.max(prevLaneIndex_, prevLaneIndexInBox_); //one of these will be -1
						//choose the egress lane with the smallest CTE
						int smallestCte = Integer.MAX_VALUE;
						for (int j = 0;  j < candidateLanes.size();  ++j) {
							Candidate c = candidateLanes.get(j);
							if (!c.approach  &&  c.cte < smallestCte) {
								chosenLane = c;
								smallestCte = c.cte;
							}
						}
						
					//else if the previous time step was associated with an approach lane then 
					}else if (prevLane.index >= 0  &&  map_.getLane(prevLane.index).isApproach()){
						//stay on lane from prev time step
						chosenLane = prevLane;
						chosenLane.cte = laneGeo_[chosenLane.index].cte(vehicle);
						chosenLane.dtsb = laneGeo_[chosenLane.index].dtsb(vehicle);
					}
					if (chosenLane == null) {
						log_.debugf("INTR", "beyond stop bar: prevDtsb = %d, no lane chosen.", prevDtsb_);
					}else {
						log_.debugf("INTR", "beyond stop bar: prevDtsb = %d, chose lane index %d", prevDtsb_, chosenLane.index);
					}
				} //endif vehicle is approaching
		
				//if CTE of the chosen lane is less than the threshold distance then indicate success
				if (chosenLane != null  &&  chosenLane.cte < cteThreshold_) {
					success = true;
				}

			} //endif multiple candidates found
			log_.debugf("INTR", "computeGeometry: detailed screening complete with success = %b, prevDtsb_ = %d, prev index = %d", success, prevDtsb_, prevLane.index);
		
			// ----- final polish - make sure that the candidate lane we identified passes a final sanity check
			
			//if we found a candidate then
			if (success) { //guarantees chosenLane is defined
				//if the candidate is an egress lane then
				if (!chosenLane.approach) {
					//we were previously on an approach then
					if (prevLane.approach) {
						//store the previous DTSB and current lane's DTSB permanently as the baseline 
						// (since DTSB on egress will still have to get more negative)
						lastApproachDtsb_ = laneGeo_[prevLane.index].dtsb(vehicle);
						firstEgressDtsb_ = chosenLane.dtsb;
						log_.infof("INTR", "///// Changing from approach to egress at lane (index %d) DTSB = %d cm", chosenLane.index, chosenLane.dtsb);
					}
					
				//else (approach lane)
				}else {
					//if the candidate's DTSB is way different from the DTSB used in the previous time step then
					int diff = prevDtsb_ - chosenLane.dtsb; //TODO changed -20 to -100 not sure what the cutoff should be
					if (prevLane.index >= 0  &&  (diff < -100 ||  diff > largeMovement_)) { //allow for drift in position signal or slow time step processing
						
						//if the candidate is different from the lane used in the previous time step then
						if (chosenLane.index != prevLane.index) {
							//if the previously used lane is still in the candidate list then
							boolean found = false;
							for (int j = 0;  j < candidateLanes.size();  ++j) {
								if (candidateLanes.get(j).index == prevLane.index) {
									found = true;
									log_.debugf("INTR", "computeGeometry: DTSB jumped by %d cm; switched from lane %d to %d", diff, chosenLane.index, prevLane.index);
									break;
								}
							}
							if (found) {
								//indicate that this previously used one is our candidate
								chosenLane.index = prevLane.index;
								//if its CTE > threshold distance then
								if (laneGeo_[chosenLane.index].cte(vehicle) > cteThreshold_) {
									//indicate failure
									success = false;
									log_.debugf("INTR", "computeGeometry failed. DTSB jumped by %d cm, but prev lane (%d) had large CTE", diff, chosenLane.index);
								}
							//else
							}else {
								//indicate failure
								success = false;
								log_.debugf("INTR", "computeGeometry failed. DTSB jumped by %d cm, so reverted to prev lane (%d)", diff, chosenLane.index);
							}
						//else (candidate lane is same as the previous lane)
						}else {
							//indicate failure
							success = false;
							log_.debugf("INTR", "computeGeometry failed. Chose same lane (%d) but DTSB jumped by %d cm", chosenLane.index, diff);
						}
					} //endif DTSB is way different
				} //endif approach lane
			} //endif found a candidate

		} //endif at least one candidate found
		
		//if no lanes found but we are wandering around in the stop box then
		if (!success  &&  prevDtsb_ < 0  &&  prevLane.approach) {

			//if the DTSB is just a little negative then
			if (prevDtsb_ > -Constants.THRESHOLD_DIST) {

				//stay with the one who brung ya (previous approach lane)
				chosenLane = prevLane;
				chosenLane.cte = laneGeo_[chosenLane.index].cte(vehicle);
				chosenLane.dtsb = laneGeo_[chosenLane.index].dtsb(vehicle);
				success = true;
				log_.info("INTR", "computeGeometry: in stop box, no other lanes available so staying with approach lane.");
			}else {
				log_.info("INTR", "computeGeometry: deep into stop box, no lanes available; not going to force any.");
			}
		}
		
		//at this point if no lane was successfully chosen then chosenLane.index will be -1 (no need to do anything here)
		
		//store the new values for use in other methods and for this method in the next time step
		if (success) {
			laneIndex_ = chosenLane.index;
			laneDtsb_ = chosenLane.dtsb;
			cte_ = chosenLane.cte;
			prevLaneIndex_ = chosenLane.index;
			prevLaneApproach_ = chosenLane.index >= 0  &&  map_.getLane(chosenLane.index).isApproach();
			log_.debugf("INTR", "computeGeometry succeeded. lane index = %d, lane ID = %d, laneDtsb = %d cm, CTE = %d cm", 
						laneIndex_, laneId(), laneDtsb_, cte_);
		}else {
			laneIndex_ = -1;
			//if we are substantially through the intersection, then keep the dtsb negative to reflect that
			if (prevDtsb_ < -Constants.THRESHOLD_DIST) {
				laneDtsb_ = laneGeo_[prevLaneIndexInBox_].dtsb(vehicle);
			}else {
				laneDtsb_ = Integer.MAX_VALUE;
			}
			cte_ = Integer.MAX_VALUE;
			prevLaneIndex_ = -1;
			prevLaneApproach_ = false;
		}
		//compute the DTSB - need to do this here so we'll have it for comparison in the next call to this method
		computeDtsb();

		return success;
	}
	
	/**
	 * vehicle has a single associated lane : ID of the associated lane
	 * no single lane can be associated to the vehicle position : -1
	 * 
	 * Note: this is the ID of the lane (as specified in the MAP message), and not necessarily its storage index
	 */
	public int laneId() {
		if (laneIndex_ >= 0) {
			return map_.getLane(laneIndex_).id();
		}
		return -1;
	}

	/**
	 * laneId represents an approach lane : true
	 * laneId does not represent an approach lane : false
     */
	public boolean isApproach(int laneId) {
		for (int i = 0;  i < map_.numLanes();  ++i) {
			Lane lane = map_.getLane(i);
			int id = lane.id();
			if (id == laneId) {
				return lane.isApproach();
			}
		}
		return false;
	}
	
	/**
	 * vehicle has a single associated lane : distance to the approach lane's stop bar in meters
	 * no single lane can be associated to the vehicle position : a very large number
	 * 
	 * Note: distance to stop bar is positive if vehicle is associated with an approach lane and the intersections has not
	 * been reached.  It will be negative if associated lane is egress, or if associated lane is approach but the vehicle
	 * has crossed the stop bar (is in the intersections box).
	 */
	public double dtsb() {
		
		//convert the stored value to meters
		return 0.01*(double)prevDtsb_;
	}
	
	/**
	 * vehicle has a single associated lane : cross-track error in cm
	 * no single lane can be associated to the vehicle position : a very large number
	 * 
	 * Note: cte is always positive, represents the lateral distance from the vehicle to the nearest line segment of
	 * the associated lane.  If the lane type is approach and dtsb < 0, then the line segment adjacent to the stop bar
	 * will be extended through the intersections to determine cte.
	 */
	public int cte() {
		return cte_;
	}
	
	/**
	 * num lanes > 1 : greatest distance between any two lane stop bars, meters
	 * num lanes <= 1 : 0
	 */
	public double stopBoxWidth() {
		return stopBoxWidth_;
	}

	public int getLaneIndex()
	{
		return laneIndex_;
	}
	
	public void setLaneIndex(int idx)
	{
		laneIndex_=idx;
	}
	//////////////////
	// member elements
	//////////////////
	
	//A note about distance to stop bar (DTSB) as used in this class:
	//Every lane has a stop bar, defined by its node 0.  If the vehicle is associated with that lane, then it has a laneDtsb_, which is the
	//distance from the vehicle along the segments of that lane to its stop bar.  On approach to the intersections this is simply the DTSB
	//for the whole scenario.  It gets tricky when we pass that stop bar and proceed through the intersections (through the stop box) and
	//on to the egress lane, however.  Once we cross that approach lane stop bar the DTSB for the scenario needs to be negative, and continue
	//to grow to more negative values as the vehicle proceeds farther downtrack from that approach stop bar.  Eventually, the vehicle will
	//associate with an egress lane.  Its distance to that stop bar will at first be small (positive), then grow as we drive farther away from
	//the stop box.  Just using this new lane's DTSB clearly is not what we want for the scenario's DTSB, which needs to become larger negative.
	//Therefore, at the point of transition between the approach lane and the egress lane (probably somewhere in the stop box), we store the
	//lastApproachDtsb_ and the firstEgressDtsb_ to allow proper math in the computation of the scenario's overall DTSB from that point on.
	
	/**
	 * vehicle has a single associated lane : stores distance to the approach lane's stop bar in cm
	 * no single lane can be associated to the vehicle position : stores a very large number
	 */
	private void computeDtsb() {
		int result;
		
		//if we are associated with a lane then
		if (laneIndex_ >= 0) {
			//if it's an approach lane then
			if (prevLaneApproach_) {
				//use its DTSB
				result = laneDtsb_;
			//else
			}else {
				//result = approach baseline - (reported DTSB - egress baseline)
				result = lastApproachDtsb_ - (laneDtsb_ - firstEgressDtsb_);
			}
		//else
		}else {
			//if lane DTSB is negative (we are lost in the stop box somewhere, about to depart the intersection) then
			if (laneDtsb_ < 0) {
				//retain the negative lane DTSB
				result = laneDtsb_;
			}else {
				//set it to a very large (positive) number
				result = Integer.MAX_VALUE;
			}
		}
		
		//store the DTSB as the previous time step's value for the next time step's call to computeGeometry()
		prevDtsb_ = result;
	}
	// TODO we need to determine how to set the laneIndex based on the returned lane id
	// Maybe just return the index and then set that
	private int								laneIndex_;			//storage vector index of the associated lane
	private int								laneDtsb_;			//vehicle's distance to stop bar of the currently associated lane, cm
	private int								cte_;				//vehicle's current cross-track error, cm
	private int								cteThreshold_;		//threshold beyond which a CTE is considered bogus, cm
	private int								prevLaneIndex_;		//index (not ID) of the lane successfully associated on prev time step; -1 if none
	private int								prevLaneIndexInBox_;//index of former approach lane (only used when deep in box with no associated lane)
	private boolean							prevLaneApproach_;	//was the associated lane in the previous time step an approach lane?
	private int								prevDtsb_;			//route DTSB from the previous time step, cm
	private int								lastApproachDtsb_;	//DTSB on the final time step that we were associated with the approach lane
	private int								firstEgressDtsb_;	//DTSB on the first time step that we were associated with the egress lane
	private MapMessage						map_;				//the latest MAP message that we've seen
	private LaneGeometry[]					laneGeo_;			//array of geometries for each lane specified in the MAP message
	private double							stopBoxWidth_;		//farthest distance between any two lanes' stop bars, m
	private double							timeStep_;			//duration of a time step, sec
	private int								largeMovement_;		//unrealistically large distance to travel in one time step, cm
	private static ILogger log_ = LoggerManager.getLogger(IntersectionGeometry.class);
}
