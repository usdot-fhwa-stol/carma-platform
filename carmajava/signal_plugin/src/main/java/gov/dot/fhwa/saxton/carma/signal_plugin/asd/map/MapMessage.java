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

package gov.dot.fhwa.saxton.carma.signal_plugin.asd.map;

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Lane;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.Location;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.ILogger;
import gov.dot.fhwa.saxton.carma.signal_plugin.logger.LoggerManager;

import java.util.List;
import java.util.Vector;

/**
 * This class represents an FHWA-formatted MAP (GID) message that has been received by the ASD OBU.  It can parse the message
 * content and deliver its various data elements.
 * 
 * @author starkj
 *
 */
public class MapMessage implements IAsdMessage {
	
	public enum LaneDirection {
		APPROACH,
		EGRESS,
		BARRIER
	};

	public MapMessage() {
		lane_ = new Vector<Lane>();
	}

	
	/**
	 * always : content version of this message (per FHWA spec)
	 * 
	 * Note: for performance reasons, this method ASSUMES that parse() has already been called and was successful. 
	 * Therefore, no data validation is being done here.
	 */
	public int getContentVersion() {
		
		return contentVersion_;
	}
	
	/**
	 * always : ID number of the intersections, as broadcast in the MAP message
	 * 
	 * Note: for performance reasons, this method ASSUMES that parse() has already been called and was successful. 
	 * Therefore, no data validation is being done here.
	 */
	@Override
	public int getIntersectionId() {
		
		return intersectionId_;
	}

	/**
	 * always : number of approach & egress lanes defined in this message
	 * 
	 * Note: for performance reasons, this method ASSUMES that parse() has already been called and was successful. 
	 * Therefore, no data validation is being done here.
	 */
	public int numLanes() {
		
		return lane_.size();
	}

	/**
	 * index is a valid lane index for this intersections : object that describes the lane
	 * index is invalid : exception
	 * 
	 * Note: index refers to internal storage position (0 <= index < numLanes()), and is NOT necessarily the same as the MAP message's ID
	 * of the lane.
	 * 
	 * Note: for performance reasons, this method ASSUMES that parse() has already been called and was successful. 
	 * Therefore, no data validation is being done here.
	 */
	public Lane getLane(int index) {
		
		return lane_.get(index);
	}
	
	/**
	 * always : intersections reference point
	 * 
	 * Note: for performance reasons, this method ASSUMES that parse() has already been called and was successful. 
	 * Therefore, no data validation is being done here.
	 */
	public Location getRefPoint() {
		
		return refPoint_;
	}

	/**
	 * prints a synopsis of the message content to the log file for human reading
	 */
	public void logSummary() {
		log_.infof("MAP", "MAP message: intersections ID = %d, content version = %d, ref lat = %f, ref lon = %f",
					intersectionId_, contentVersion_, refPoint_.lat(), refPoint_.lon());
		int approach = 0;
		int egress = 0;
		for (Lane lane : lane_){
			if (lane.isApproach()){
				++approach;
			}else{
				++egress;
			}
			Location[] nodes = lane.getNodes();
			int sum = 0;
			for (int i = 1;  i < nodes.length;  ++i){
				sum += nodes[i].distanceFrom(nodes[i-1]);
			}
			log_.infof("MAP", "MAP message: %s lane %d is %d cm wide, has %d nodes, and is %.1f m long", lane.isApproach() ? "APPROACH" : "EGRESS", lane.id(), lane.width(), nodes.length, sum/100.0);
		}
		log_.infof("MAP", "MAP message: has a total of %d approach and %d egress lanes", approach, egress);
	}
	

	// CARMA required setters, used for conversion from 2016 MAP to 2009 MAP

	public void setElevationsPresent(boolean present) {
		this.elevationsPresent_ = present;
	}

	public void setOffsetsInDm(boolean value) {
		this.offsetsInDm_ = value;
	}

	public void setIntersectionId(int intersectionId) {
		this.intersectionId_ = intersectionId;
	}

	public void setRefPoint(Location refPoint) {
		this.refPoint_ = refPoint;
	}

	public void setLanes(List<Lane> lanes) {
		lane_ = new Vector<>(lanes);
	}

	public void setContentVersion(int version) {
		contentVersion_ = version;
	}
	
	private boolean				elevationsPresent_;		//will elevation data be present in reference point and node definitions?
	private boolean				offsetsInDm_;			//are lane node offsets stored in decimeters? (false indicates centimeters)
	private int					contentVersion_;		//the version sequence # of the MAP content published by the intersections RSU
	private int					intersectionId_;		//the regional ID number of this intersections
	private Location			refPoint_;				//the reference point
	private Vector<Lane>		lane_;
	private static ILogger		log_ = LoggerManager.getLogger(MapMessage.class);
}
