package gov.dot.fhwa.saxton.carma.signal_plugin.asd.map;

import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.Lane;
import gov.dot.fhwa.saxton.glidepath.asd.Location;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

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
	 * buf length <= 4 : false (4 bytes is just the msg header).  Otherwise...
	 * any data element doesn't fit the FHWA spec* : false
	 * all data elements have plausible values : true (and elements stored internally for future retrieval)
	 * 
	 * Caution: this must be the first method called on any instance of this class. Until it is parsed, which validates
	 * the content, none of the other methods will be meaningful.  And to maximize speed of execution, none of them
	 * attempt to verify that the parsing has first been performed.  It is up to the user to control this interaction.
	 * If parse() returns false, then do not call any of the remaining methods on that object.
	 * 
	 * *FHWA spec used is "Inteface Control Document for the Signal Phase and Timing and Related Messages for V-I Applications",
	 * contract no. DTFH61-06-D-00007 by Battelle & Texas A&M University, doc no. 60606-18A Final, dated June 2013, section 3.2.2.
	 * 
	 * @param buf - buffer of raw input stream coming from the ASD hardware, which may contain a useful message
	 */
	public boolean parse(byte[] buf) {
		
		boolean			failure = false;
		int				pos = 1; //read position in the buffer (first byte is pos 0, which indicates MAP message type)
		Lane			curLane = null; //the lane object currently being populated
		LaneDirection	laneType = LaneDirection.BARRIER;
		
		//store message content version
		contentVersion_ = (int)buf[pos++];
		
		//store overall payload length, and initialize the content length accumulator, which will ensure we don't read past the end of message
		int payloadLength = get2Bytes(buf, pos);
		pos += 2;
		int accumulator = 0;
		//log_.debugf("MAP", "MAP message version %d has payload length %d", contentVersion_, payloadLength);
		
		//begin loop on message objects, while no content failure and we haven't read the full length of the payload
		while (!failure  &&  accumulator < payloadLength) {
		
			//get the object identifier and the length of the object (next two bytes); add object length to the length accumulator
			int objectId = buf[pos++];
			int objLength = 0;
			if ((objectId & 0x000000ff) != 0xff){ //the 0xff flag doesn't come with a length byte
				objLength = buf[pos++];
				++accumulator; //account for the object length byte
			}
			accumulator += objLength + 1; //adds object payload plus 1 byte for the object ID itself
			
			//if accumulator is larger than the advertised message length then indicate failure and break out of loop
			if (accumulator > payloadLength) {
				log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d has length %d, putting us over payload length of %d", objectId, objLength, payloadLength);
				failure = true;
				break;
			}
			//log_.debugf("MAP", "MAP msg objectId = %d, objLength = %d", objectId, objLength);
		
			//switch on the object identifier
			switch(objectId & 0x000000ff) {
		
			case MESSAGE_ATTRIBUTES:
				//if size word isn't 1 indicate failure
				if (objLength != 1) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//if the geometry bit isn't set then indicate failure
					int attrib = buf[pos++];
					if ((attrib & 0x04) == 0) {
						log_.warnf("MAP", "Parse failure on presumed MAP message: geometry data not indicated. attrib = %04x", attrib);
						failure = true;
					}else {
						//store the offset resolution
						offsetsInDm_ = (attrib & 0x02) > 0;
						//store whether elevation data will be included
						elevationsPresent_ = (attrib & 0x01) > 0;
						//log_.debugf("MAP", "parse: MAP attributes are offsets in %s, elevationsPresent = %b",
						//			offsetsInDm_ ? "dm" : "cm", elevationsPresent_);
					}
				}
				break;

			case INTERSECTION_ID:
				//if size word isn't 4 indicate failure
				if (objLength != 4) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//store the ID
					intersectionId_ = get4Bytes(buf, pos);
					pos += 4;
				}
				break;
		
			case REFERENCE_POINT:
				//if size is 8 and no elevation present or size is 10 with elevation present then
				if ((objLength == 8  &&  !elevationsPresent_)  ||  (objLength == 10 &&  elevationsPresent_)) {
					//read and convert lat & lon
					int rawLat = get4Bytes(buf, pos);
					pos += 4;
					int rawLon = get4Bytes(buf, pos);
					pos += 4;
					refPoint_ = new Location((double)rawLat/1.0e7, (double)rawLon/1.0e7);
					
					//if elevation is present then
					if (elevationsPresent_) {
						//skip past elevation data (we won't use that here)
						pos += 2;
					}
				//else
				}else {
					//indicate failure
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}
				break;
		
			case APPROACH_EGRESS_BARRIER:
				//if size is not 1 then indicate failure
				if (objLength != 1) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//read the indicator and set the control variable (which will be used for other cases in this loop)
					int ind = buf[pos++];
					switch (ind) {
					case 1:  laneType = LaneDirection.APPROACH;	break;
					case 2:  laneType = LaneDirection.EGRESS;	break;
					case 3:  laneType = LaneDirection.BARRIER;	break;
					default:
						log_.warnf("MAP", "Parse failure on presumed MAP message: lane type indicator = %d", ind);
						failure = true;
					}
				}
				break;
				
			case LANE:
				//if size is not 2 then indicate failure
				if (objLength != 2) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//if indicator is either approach or egress then
					if (laneType == LaneDirection.APPROACH  ||  laneType == LaneDirection.EGRESS) {
						//create a new lane object and store this indicator for it
						curLane = new Lane();
						curLane.setApproach(laneType == LaneDirection.APPROACH);
						lane_.add(curLane);
					}
					//read the lane ID and the lane type
					int laneNum = buf[pos++];
					int type = buf[pos++];
					curLane.setId(laneNum);
					
					//Note: for now, Glidepath project assumes no non-motorized lanes are defined for the test intersections.
					// In future if other intersections are used, this will have to be made more intelligent to ignore other
					// types of lanes during the reading, or to read them in and ensure that they are ignored in the geometry calcs.
					if (type != 1) { //for motorized vehicle
						log_.warnf("MAP", "MAP message failure: lane %d type is %d", laneNum, type);
						failure = true;
					}
				}
				break;
		
			case LANE_ATTRIBUTES:
				//if size is not 2 then indicate failure
				if (objLength != 2) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					int attrib = get2Bytes(buf, pos);
					pos += 2;
					//if lane type indicator is not barrier then
					if (laneType != LaneDirection.BARRIER) {
						//if bits 0, or 8-12 are set then issue a warning (unsafe for Glidepath experiment); 
						// this is not a message parsing failure however
						if ((attrib & 0x1f01) != 0){
							log_.warnf("MAP", "MAP message TRAVEL SAFETY WARNING: lane %d attributes %04x indicate unsafe for Glidepath", curLane.id(), attrib);
						}
						//store the attribute word in the lane object
						curLane.setAttributes(attrib);
					}
				}
				break;
		
			case BARRIER_ATTRIBUTES:
				if (objLength != 2) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//skip past the indicated number of bytes - not used in Glidepath
					pos += 2;
				}
				break;
		
			case WIDTH:
				//if size is not 2 then indicate failure
				if (objLength != 2) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					int width = get2Bytes(buf, pos);
					pos += 2;
					//if lane type indicator is not a barrier then
					if (laneType != LaneDirection.BARRIER) {
						//store the width in the lane object
						curLane.setWidth(width);
					}
				}
				break;
		
			case NODE_LIST:
				//if size < 10 then indicate failure (allows room for two nodes)
				if (objLength < 10) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//if lane type indicator is barrier then
					if (laneType == LaneDirection.BARRIER) {
						//skip past the bytes in this object
						pos += objLength;
					//else
					}else {
						//parse all of the nodes describing this lane and populate the lane object
						boolean success = parseNodeList(buf, pos, objLength, curLane);
						pos += objLength;
						if (success) {
							//log_.debugf("MAP", "Lane ID %d now contains %d nodes", curLane.id(), curLane.getNodes().length);
						}else {
							failure = true; //don't clear this flag on success, because it might be set elsewhere
						}
					} 
				}
				break;
		
			case CONNECTION:
				//if size is not 2 then indicate failure
				if (objLength != 2) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//skip past the next two bytes
					pos += 2;
				}
				break;
		
			case REFERENCE_LANE:
				//if size is not 3 then indicate failure
				if (objLength != 3) {
					log_.warnf("MAP", "Parse failure on presumed MAP message: object ID %d length is %d", objectId, objLength);
					failure = true;
				}else {
					//skip past the next 3 bytes
					pos += 3;
				}
				break;
		
			case END_OF_MESSAGE:
				//indicate completion of loop
				break;
		
			default:
				//indicate message failure
				log_.warnf("MAP", "Parse failure on presumed MAP message: objectId %d unknown", objectId);
				failure = true;
			} //end switch
		
		} //end loop on message objects
		
		//if parsing succeeded then
		if (!failure) {
			//if reference point is defined then
			if (refPoint_.lat() != 0.0  &&  refPoint_.lon() != 0.0) {
				
				//loop through each lane 
				int numApproach = 0;
				int numEgress = 0;
				for (Lane lane : lane_) {
					//count the number of approach and egress lanes
					if (lane.isApproach()) {
						++numApproach;
					}else {
						++numEgress;
					}
					
					//if it has < 2 nodes then
					if (lane.getNodes().length < 2) {
						//indicate failure
						failure = true;
						log_.warn("MAP", "MAP message failed because lane has fewer than 2 nodes.");
					//else if it is missing width or attributes then
					}else if (lane.width() == 0  ||  lane.attributes() == 0) {
						//indicate failure
						failure = true;
						log_.warn("MAP", "MAP message failed because lane width or attributes not defined.");
					}
				}
				
				//if there is not at least one approach lane then
				if (numApproach == 0) {
					//indicate failure
					failure = true;
					log_.warnf("MAP", "MAP message failed due to lack of approach lanes: %d approaches, %d egresses", numApproach, numEgress);
				}
			}else {
				failure = true;
				log_.warn("MAP", "MAP message failed due to lack of reference point.");
			}
		} 
		
		if (!failure) {
			logSummary();
		}
		
		return !failure;
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
	
	//////////////////
	// private members
	//////////////////
	
	private int	get2Bytes(byte[] b, int p) {
		int res = ((b[p] << 8) & 0x0000ff00)  |  (b[p+1] & 0x000000ff);

		//ensure negatives will still be represented as such
		if ((res & 0x00008000) != 0) {
			res |= 0xffff0000;
		}
		return res;
	}
	
	private int get4Bytes(byte[] b, int p) {
		int res = ((b[p] << 24) & 0xff000000)  |  ((b[p+1] << 16) & 0x00ff0000)  |  ((b[p+2] << 8) & 0x0000ff00)  |  (b[p+3] & 0x000000ff);
		return res;
	}
	
	private boolean parseNodeList(byte[]b, int p, int len, Lane lane) {
		int bufLoc = 0;
		int eastOffset = 0;
		int northOffset = 0;

		//get the attribute byte and decode it
		int attrib = b[p + bufLoc++];
		boolean hasWidth = (attrib & 0x01) != 0;
		boolean packed   = (attrib & 0x02) != 0;
		//log_.debugf("MAP", "parseNodeList for lane %d: hasWidth = %b, packed = %b", lane.id(), hasWidth, packed);
		
		//if data are packed, then both the elevation & width data need to be present or absent, since they 
		//share a byte of the data, so check if these presences are not in sync
		if (packed  &&  (elevationsPresent_ ^ hasWidth)) {
			log_.warnf("MAP", "Parse failure on presumed MAP message: node list is packed, but elevations present = %b and hasWidth = %b", elevationsPresent_, hasWidth);
			return false;
		}
		
		//loop on nodes until we've run out of buffer
		while (bufLoc < len){
		
			//if nodes are packed then
			if (packed) {
				byte byte1 = b[p + bufLoc++];
				byte byte2 = b[p + bufLoc++];
				byte byte3 = b[p + bufLoc++];
				
				//build the east offset from the first 3 nibbles
				eastOffset = ((byte1 << 4) & 0x000007f0)  |  ((byte2 >>> 4) & 0x0000000f);
				//build the north offset from the next 3 nibbles
				northOffset = ((byte2 << 8) & 0x00000700) | (byte3 & 0x000000ff);
				
				//if these are negative numbers, need to manually take the 2s complement for correct representation
				if ((byte1 & 0x00000080) > 0) {
					eastOffset = -((~eastOffset & 0x000007ff) + 1);
				}
				if ((byte2 & 0x00000008) > 0) {
					northOffset = -((~northOffset & 0x000007ff) + 1);
				}
				
				//discard next two bytes (if elevations are present, which we don't use)
				if (elevationsPresent_) {
					bufLoc += 2;
				}
				//if width is included then discard the next byte
				if (hasWidth) {
					++bufLoc;
				}
				
			//else
			}else {
				//store east offset from first two bytes
				eastOffset = get2Bytes(b, p+bufLoc);
				bufLoc += 2;
				//store north offset from next two bytes
				northOffset = get2Bytes(b, p+bufLoc);
				bufLoc += 2;
				//discard next two bytes (if elevations are present, which we don't use)
				if (elevationsPresent_) {
					bufLoc += 2;
				}
				//if width is included then discard the next two bytes
				if (hasWidth) {
					bufLoc += 2;
				}
			}
			//log_.debugf("MAP", " storing eastOffset = %d, northOffset = %d (units defined earlier)", eastOffset, northOffset);
			
			//store the offsets with the correct units of measure
			if (offsetsInDm_) {
				lane.addNodeDm(refPoint_, eastOffset, northOffset);
			}else{
				lane.addNodeCm(refPoint_, eastOffset, northOffset);
			}
				
		}

		return true;
	}
	
	
	private boolean				elevationsPresent_;		//will elevation data be present in reference point and node definitions?
	private boolean				offsetsInDm_;			//are lane node offsets stored in decimeters? (false indicates centimeters)
	private int					contentVersion_;		//the version sequence # of the MAP content published by the intersections RSU
	private int					intersectionId_;		//the regional ID number of this intersections
	private Location			refPoint_;				//the reference point
	private Vector<Lane>		lane_;
	private static ILogger		log_ = LoggerManager.getLogger(MapMessage.class);
	
	private final int			MESSAGE_ATTRIBUTES			= 0x01;
	private final int			INTERSECTION_ID				= 0x02;
	private final int			REFERENCE_POINT				= 0x03;
	private final int			APPROACH_EGRESS_BARRIER		= 0x04;
	private final int			LANE						= 0x05;
	private final int			LANE_ATTRIBUTES				= 0x06;
	private final int			BARRIER_ATTRIBUTES			= 0x07;
	private final int			WIDTH						= 0x08;
	private final int			NODE_LIST					= 0x09;
	private final int			CONNECTION					= 0x0a;
	private final int			REFERENCE_LANE				= 0x0b;
	private final int			END_OF_MESSAGE				= 0xff;
}
