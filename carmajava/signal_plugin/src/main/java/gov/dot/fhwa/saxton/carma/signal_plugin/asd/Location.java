package gov.dot.fhwa.saxton.carma.signal_plugin.asd;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;


/**
 * This class represents a point on the surface of the earth.  It uses a flat-earth
 * approximation, so it is only accurate for points that are a fairly short distance 
 * away from a given reference point. 
 * 
 * Note:  the reference point as used here may not necessarily be the "reference point"
 * for a given intersections that is used for MAP (GID) messages for traffic management.
 * 
 * @author starkj
 *
 */
public class Location {
	
	/**
	 * always : stores the east & north offsets from a given reference point
	 * 
	 * @param ref   : the reference point
	 * @param east	: distance east of the reference point, cm
	 * @param north : distance north of the reference point, cm
	 */
	public Location(Location ref, int east, int north){
		setOffset(ref, east, north);
	}
	
	/**
	 * always : stores lat/lon for the point as given
	 * 
	 * @param lat : the given latitude, in deg
	 * @param lon : the given longitude, in deg
	 */
	public Location(double lat, double lon) {
		setLatLon(lat, lon);
	}

	/**
	 * always : stores lat/lon for the point as given
	 * 
	 * @param lat : the given latitude, in deg
	 * @param lon : the given longitude, in deg
	 */
	public void setLatLon(double lat, double lon) {
		lat_ = lat;
		lon_ = lon;
	}

	/**
	 * always : stores the east & north offsets from a given reference point
	 * 
	 * @param ref   : the reference point
	 * @param east	: distance east of the reference point, cm
	 * @param north : distance north of the reference point, cm
	 */
	public void setOffset(Location ref, int east, int north){
		lat_ = ref.lat() + (double)north/CM_PER_DEG;
		lon_ = ref.lon() + (double)east/(CM_PER_DEG*Math.cos(ref.lat()*RAD_PER_DEG));
	}
	
	/**
	 * always : latitude, in deg
	 */
	public double lat() {
		return lat_;
	}
	
	/**
	 * always : longitude, in deg
	 */
	public double lon() {
		return lon_;
	}
	
	/**
	 * always : straight-line distance in cm from location p to this location
	 * Note: assumes flat earth model is sufficient within the confines of the geometry represented.
	 */
	public int distanceFrom(Location p) {
		
		double de = (p.lon() - lon_)*CM_PER_DEG*Math.cos(p.lat()*RAD_PER_DEG);
		double dn = (p.lat() - lat_)*CM_PER_DEG;
		double dist = Math.sqrt(de*de + dn*dn);
		
		return (int)(dist + 0.5);
	}
	
	/**
	 * always : eastern component of the distance in cm from p to this location
	 */
	public int eastOffsetFrom(Location p) {
		
		double offset = (lon_ - p.lon())*CM_PER_DEG*Math.cos(p.lat()*RAD_PER_DEG);
		
		return (int)(offset + 0.5);
	}
	
	/**
	 * always : northern component of the distance in cm from p to this location
	 */
	public int northOffsetFrom(Location p) {
		
		double offset = (lat_ - p.lat())*CM_PER_DEG;
		
		return (int)(offset + 0.5);
	}
	
	//////////////////
	// private members
	//////////////////
	
	private double				lat_;	//latitude, in deg
	private double				lon_;	//longitude, in deg
	private static final double	CM_PER_DEG = Constants.RADIUS_OF_EARTH_METERS*Constants.PI/1.8;
	private static final double	RAD_PER_DEG = Constants.PI/180.0;
}
