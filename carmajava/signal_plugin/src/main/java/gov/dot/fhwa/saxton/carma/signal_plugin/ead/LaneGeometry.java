package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import gov.dot.fhwa.saxton.glidepath.appcommon.Constants;
import gov.dot.fhwa.saxton.glidepath.asd.Lane;
import gov.dot.fhwa.saxton.glidepath.asd.Location;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;

import java.util.Vector;

// Represents the geometry of a given lane in a Cartesian coordinate system where the
// lane's reference point (per MAP/GID spec) is the origin.  Coordinates are in east (+x)
// and north (+y) centimeters from the origin.  The lane's geometry is composed of points
// that represent the MAP nodes (points) and line segments that connect those points such that
// they are physically laid out and indexed like this:
//
// Ref pt    P0     S0      P1  S1   P2   S2    P[N-1]  S[N-1]   PN
//    +       +--------------+--------+-------...-+--------------+
//
// where P0 is the lane's stop bar.  Note that the reference point may not necessarily be
// in the center of the stop box, or even in the stop box at all.  This diagram simply
// clarifies that it is near P0 somewhere, and not part of the lane centerline.
//
// The term "downtrack" refers to items closer to the stop bar than the location in question
// (uses the context of a vehicle traveling toward the intersections, so this seems backwards
// if referring to an egress lane).

public class LaneGeometry {
	
	/**
	 * always : adds geometric information to the given lane
	 * 
	 * @param ref - the intersections's reference point (as defined in the MAP message)
	 * @param lane - a Lane object from the MAP message
	 * @throws Exception if the lane includes two adjacent nodes that are in the same location
	 * 
	 * Note: it would save memory and some cpu time during construction to only store line segments 
	 * (better yet with only one Point in a line segment), but we want to store the points redundantly
	 * here to minimize cpu time on the distance computations that happen every time step.
	 */
	public LaneGeometry(Location ref, Lane lane) throws Exception {
		points_ = new Vector<CartesianPoint2D>();
		segments_ = new Vector<LineSegment2D>();
		
		boxMaxX_ = -Integer.MAX_VALUE;
		boxMaxY_ = -Integer.MAX_VALUE;
		boxMinX_ = Integer.MAX_VALUE;
		boxMinY_ = Integer.MAX_VALUE;
		
		//store the intersections's reference point
		reference_ = ref;
		
		//loop through the nodes in the lane
		Location[] nodes = lane.getNodes();
		CartesianPoint2D p0 = null;
		//log_.debugf("INTR", "LaneGeometry constructor for lane ID = %d", lane.id());
		for (int i = 0;  i < nodes.length;  ++i) {
			
			//store it as a point in Cartesian coordinates that represent our flat Earth model
			int x = nodes[i].eastOffsetFrom(ref);
			int y = nodes[i].northOffsetFrom(ref);
			CartesianPoint2D p1 = new CartesianPoint2D((double)x, (double)y);
			points_.add(p1);
			//log_.debugf("INTR", "    node %2d x = %5d cm, y = %5d cm", i, x, y);
			
			//record the min & max x & y coordinates
			if (x < boxMinX_) boxMinX_ = x;
			if (y < boxMinY_) boxMinY_ = y;
			if (x > boxMaxX_) boxMaxX_ = x;
			if (y > boxMaxY_) boxMaxY_ = y;
			
			//construct a line segment that joins this node with the previous one
			if (i > 0) {
				LineSegment2D seg;
				try {
					seg = new LineSegment2D(p0, p1);
					segments_.add(seg);
				} catch (Exception e) {
					log_.errorf("INTR", "Attempted to define zero-length line segment between nodes %d and %d in lane ID %d", i-1, i, lane.id());
					throw e;
				}
			}
			p0 = p1;
		}
		
		//finalize the bounding rectangle for all points in this lane (add a tolerance all around)
		boxMinX_ -= 4*Constants.THRESHOLD_DIST;
		boxMinY_ -= 4*Constants.THRESHOLD_DIST;
		boxMaxX_ += 4*Constants.THRESHOLD_DIST;
		boxMaxY_ += 4*Constants.THRESHOLD_DIST;
		//log_.debugf("INTR", "Lane ID %d constructed. Bounding box = (%d, %d)..(%d, %d)", lane.id(), boxMinX_, boxMinY_, boxMaxX_, boxMaxY_);
		
		//initialize other necessary variables
		dtsb_ = Integer.MAX_VALUE;
		cte_ = Integer.MAX_VALUE;
		inputLocation_ = null;
	}
	
	/**
	 * p is within the rectangle that bounds all points in this lane : true
	 * otherwise : false
	 */
	public boolean inBoundingBox(Location latLon) {
		
		//convert the location to a Cartesian point
		CartesianPoint2D p = new CartesianPoint2D(latLon.eastOffsetFrom(reference_), latLon.northOffsetFrom(reference_));
		
		//determine if it is within the bounds
		boolean res = false;
		if (p.x() > boxMinX_  &&  p.x() < boxMaxX_) {
			if (p.y() > boxMinY_  &&  p.y() < boxMaxY_) {
				res = true;
			}
		}
		
		return res;
	}

	/**
	 * latLon is near one of the lane's line segments with index > 0 
	 * 		: distance from latLon to that segment's downtrack point + length of all downtrack segments
	 * latLon near first line segment : distance from latLon's projection onto extended first segment to stop bar point
	 * latLon is not near any of the lane's line segments : a very large (positive) number
	 * 
	 * Note: "extended first segment" means that the first line segment (segment 0) is 
	 * extended infinitely in the downtrack direction (through the intersections stop box) so that negative DTSB
	 * can be calculated if the vehicle is somewhere in the stop box.
	 * 
	 * Note: in first condition, if latLon is beside more than one of the lane's segments (may happen if the vehicle
	 * is on the inside of a curved lane), then the first segment (from the far end of the lane) with a 
	 * CTE < lane width will be used.
	 * 
	 * @return distance in cm
	 */
	public int dtsb(Location latLon) {
		
		//if the given location is different from the previously calculated location then
		if (inputLocation_ == null  ||  latLon.lat() != inputLocation_.lat()  ||  latLon.lon() != inputLocation_.lon()) {
			//perform the distance computations
			computeDistances(latLon);
			//save this location to avoid future expensive computations
			inputLocation_ = latLon;
		}
		
		return dtsb_;
	}
	
	/**
	 * always : distance from latLon to the closest point in the lane's set of line segments
	 * 
	 * @return distance in cm (is always positive) 
	 */
	public int cte(Location latLon) {

		//if the given location is different from the previously calculated location then
		if (inputLocation_ == null  ||  latLon.lat() != inputLocation_.lat()  ||  latLon.lon() != inputLocation_.lon()) {
			//perform the distance computations
			computeDistances(latLon);
			//save this location to avoid future expensive computations
			inputLocation_ = latLon;
		}
		
		return cte_;
	}
	
	/**
	 * always : the point representing the stop bar on this lane
	 */
	public CartesianPoint2D getStopBar() {
		return points_.get(0);
	}
	
	//////////////////
	// member elements
	//////////////////
	
	
	/**
	 * CTE < threshold  : calculated CTE & DTSB
	 * CTE >= threshold : calculated CTE (DTSB set very large)
	 * 
	 * Note: this will work even if the input latLon is way far from anything in the lane, but it is not expected to be
	 * called unless the input is pretty close to the lane.
	 */
	private void computeDistances(Location latLon) {
		//convert the input location to a Cartesian point
		CartesianPoint2D vehicle = new CartesianPoint2D(latLon.eastOffsetFrom(reference_), latLon.northOffsetFrom(reference_));

		//initialize DTSB to very large
		dtsb_ = Integer.MAX_VALUE;
		
		//find the lane node that the given point is closest to (need to loop through all of them because the lane may bend around)
		double minDist = Double.MAX_VALUE;
		int closestNode = -1;
		for (int i = 0;  i < points_.size();  ++i) {
			double dist = points_.get(i).distanceFrom(vehicle);
			if (dist < minDist) {
				minDist = dist;
				closestNode = i;
			}
		}
		
		try {
			//determine if the vehicle is inside either of the adjacent line segments (i.e. the perpendicular is between the end points)
			// and choose the segment with the smallest distance to the vehicle
			LineSegment2D segA = null;
			LineSegment2D segB = null;
			double distToA = -1.0; //shortestDistanceToPoint() returns -1 if it can't find an answer
			double distToB = -1.0;
			int downtrackNode = -1;
			int nodeOfInterest = -1;
			double distToDowntrackNode = 0.0;
			CartesianPoint2D vehicleTranslated = null;
			if (closestNode > 0) {
				segA = new LineSegment2D(points_.get(closestNode - 1), points_.get(closestNode));
				distToA = segA.shortestDistanceToPoint(vehicle);
			}
			if (closestNode < points_.size() - 1) {
				segB = new LineSegment2D(points_.get(closestNode), points_.get(closestNode + 1));
				distToB = segB.shortestDistanceToPoint(vehicle);
			}
			
			//if it is adjacent to one of these two segments then
			if (distToA >= 0.0  ||  distToB >= 0.0) {
				//identify the segment that it is closest to, and use that distance as the CTE
				//save the dist from our point to the next downtrack node
				//identify the next downtrack node as the node of interest
				if (distToA >= 0.0  &&  (distToA < distToB  ||  distToB < 0.0)) {
					cte_ = (int)(distToA + 0.5);
					downtrackNode = closestNode - 1;
					vehicleTranslated = segA.translateTo(vehicle); 
				}else { //distance to B must be > 0
					cte_ = (int)(distToB + 0.5);
					downtrackNode = closestNode;
					vehicleTranslated = segB.translateTo(vehicle);
				}
				//find the downtrack component of distance to the downtrack node
				distToDowntrackNode = points_.get(downtrackNode).distanceFrom(vehicleTranslated);
				nodeOfInterest = downtrackNode;
				log_.debugf("INTR", "computeDistances: adjacent to a segment; nodeOfInterest = %d, distToDowntrackNode = %.1f, cte = %d", nodeOfInterest, distToDowntrackNode, cte_);
				
			//else (it is near a point but outside both of its adjacent segments, so it is in the
			//     wedge between them on the outside of the curve, or it is beyond one end of the lane)
			}else {
				//if nearest node is 0 then
				if (closestNode == 0) {
					//compute the CTE to the extension of the first segment
					cte_ = (int)(segments_.get(0).shortestDistanceToPointExtendedSegment(vehicle) + 0.5);
					//set distance to next downtrack node as negative of the distance to node 0 (we're past the stop bar here)
					vehicleTranslated = segments_.get(0).translateToExtendedSegment(vehicle);
					distToDowntrackNode = -points_.get(0).distanceFrom(vehicleTranslated);
					log_.debugf("INTR", "computeDistances: beyond node 0. distToDowntrackNode = %.1f", distToDowntrackNode);
				//else if the nearest node is the last one in the lane then
				}else if (closestNode == points_.size() - 1) {
					//compute the CTE to the extension of the last segment
					cte_ = (int)(segments_.get(segments_.size() - 1).shortestDistanceToPointExtendedSegment(vehicle) + 0.5);
					//set distance to next downtrack node as distance to the last node
					vehicleTranslated = segments_.get(segments_.size() - 1).translateToExtendedSegment(vehicle);
					distToDowntrackNode = points_.get(points_.size() - 1).distanceFrom(vehicleTranslated);
					log_.debugf("INTR", "computeDistances: beyond final node %d. distToDowntrackNode = %.1f", closestNode, distToDowntrackNode);
				//else (we're somewhere in the middle of the lane)
				}else {
					//set the CTE as the distance to the nearest line node (this will be a decent approximation of reality)
					cte_ = (int)points_.get(closestNode).distanceFrom(vehicle);
					//set distance to next downtrack node to 0
					distToDowntrackNode = 0;
					log_.debugf("INTR", "computeDistances: in wedge outisde of node %d, cte = %d", closestNode, cte_);
				}
				nodeOfInterest = closestNode;
			}
			
			//compute the downtrack distance to the stop bar
			double sum = 0.0;
			CartesianPoint2D prevPoint = points_.get(0);
			for (int i = 1;  i <= nodeOfInterest;  ++i) {
				CartesianPoint2D curPoint = points_.get(i);
				sum += prevPoint.distanceFrom(curPoint);
				prevPoint = curPoint;
			}
			dtsb_ = (int)(sum + distToDowntrackNode + 0.5);
				
		} catch (Exception e) {
			//this should only be generated by LineSegment2D constructors, which are safe here because
			// we already vetted the points in the lane during our constructor, so don't need to do anything here.
			log_.warn("INTR", "Line segment exception in computeDistances(). No handler; continuing.");
		}
	}
	
	private int							boxMaxX_;		//eastern edge of the lane's bounding box, cm east of the reference point
	private int							boxMinX_;		//western edge of the lane's bounding box, cm east of the reference point
	private int							boxMaxY_;		//northern edge of the lane's bounding box, cm north of the reference point
	private int							boxMinY_;		//southern edge of the lane's bounding box, cm north of the reference point
	private int							dtsb_;			//computed distance to stop bar, in cm
	private int							cte_;			//computed cross-track error, in cm
	private Vector<CartesianPoint2D>	points_;		//the points in Cartesian coordinates that represent our lane nodes
	private Vector<LineSegment2D>		segments_;		//ordered such that item 0 is adjacent to the stop bar; subsequent ones are farther away
	private Location					reference_;		//the intersections's reference point (maps to Cartesian coordinate system's origin)
	private Location					inputLocation_;	//the location that distance computations have already been done for
	private static ILogger log_ = LoggerManager.getLogger(LaneGeometry.class);
}
