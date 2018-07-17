package gov.dot.fhwa.saxton.glidepath.asd;

import java.util.Vector;

/**
 * This class represents a single lane of approach/egress to/from a given intersections, per FHWA GID message specification.  It
 * models the lane's geometry as a series of nodes extending away from the intersections's reference point (center point),
 * with the first node in the list being the stop bar, and the last node being farthest away from the intersections.
 * The intersections also has other attributes to describe it.
 * 
 * @author starkj
 *
 */
public class Lane {
	
	public Lane() {
		node_ = new Vector<Location>();
	}

	/**
	 * always : adds a node represented by the east/north offsets (in cm) to the end of the known list of sequential 
	 * nodes (this node must be farther away from reference point than all of the other nodes already added)
	 * 
	 * Note: for computational speed, this will ASSUME that nodes are being added in sequential order, and will not do
	 * any validation of the data.
	 * 
	 * @param ref is the intersections's reference point
	 * @param east, north contain the cm offsets provided in the raw MAP message (i.e. distances from the previous node listed)
	 */
	public void addNodeCm(Location ref, int east, int north) {
		int num = node_.size();
		Location prev = new Location(ref.lat(), ref.lon());

		//if this will not be the first node stored for the lane then
		if (num > 0) {
			//compute its offset by adding the input offsets to the last stored node
			Location p = node_.get(num-1);
			prev.setLatLon(p.lat(), p.lon());
		}
		
		//create the location and add it to the end of the list
		Location loc = new Location(prev, east, north);
		node_.add(loc);
	}
	
	/**
	 * always : adds a node represented by the east/north offsets (in dm) to the end of the known list of sequential 
	 * nodes (this node must be farther away from reference point than all of the other nodes already added)
	 * 
	 * Note: for computational speed, this will ASSUME that nodes are being added in sequential order, and will not do
	 * any validation of the data.
	 * 
	 * @param ref is the intersections's reference point
	 * @param east, north contain the dm offsets provided in the raw MAP message (i.e. distances from the previous node listed)
	 */
	public void addNodeDm(Location ref, int east, int north) {
		//convert inputs to cm and add nodes in those units
		int eastCm = 10*east;
		int northCm = 10*north;
		addNodeCm(ref, eastCm, northCm);
	}
	
	/**
	 * always : returns an array of nodes that describe the centerline of the lane extending away from the reference point
	 * 
	 * Note: locations are offsets to the REFERENCE POINT, which is different from the way they appear in the MAP message.
	 */
	public Location[] getNodes() {
		
		Location[] rtn = new Location[node_.size()];
		rtn = node_.toArray(rtn);
		return rtn;
	}
	
	/**
	 * always : stores the ID number for this lane
	 */
	public void setId(int id) {
		
		laneId_ = id;
	}
	
	/**
	 * always : ID number of the lane
	 */
	public int id() {
		
		return laneId_;
	}
	
	/**
	 * always : stores width of the lane in cm
	 */
	public void setWidth(int w) {
		
		width_ = w;
	}
	
	/**
	 * always : width of the lane in cm
	 */
	public int width() {
		
		return width_;
	}
	
	/**
	 * always : stores the attribute word
	 */
	public void setAttributes(int a) {
		
		attributes_ = a;
	}
	
	/**
	 * always : the attribute word for the lane (meaning to be interpreted per FHWA spec)
	 */
	public int attributes() {
		
		return attributes_;
	}
	
	/**
	 * always : stores whether or not this lane is approaching the intersections (if not, then it is an egress lane)
	 */
	public void setApproach(boolean app) {
		
		isApproach_ = app;
	}
	
	/**
	 * always : indicates whether or not this lane is approaching the intersections (if not, then it is an egress lane)
	 */
	public boolean isApproach() {
		
		return isApproach_;
	}
	
	//////////////////
	// class members
	//////////////////
	
	private boolean						isApproach_;	//is this an approach lane? (false if it is an egress lane)
	private int							laneId_;		//ID number of the lane
	private int							width_;			//lane width in cm
	private int							attributes_;	//raw attributes bitflags
	private Vector<Location>			node_;			//ordered with stop bar at [0] then moving sequentially farther away
}
