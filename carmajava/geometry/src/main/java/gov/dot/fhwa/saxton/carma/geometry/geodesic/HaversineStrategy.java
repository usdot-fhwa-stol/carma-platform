package gov.dot.fhwa.saxton.carma.geometry.geodesic;

/**
 * Created by mcconnelms on 9/12/17.
 */
public class HaversineStrategy implements IDistanceStrategy{
  protected final double r = 6372800.0; // Average radius of the earth in meters

  @Override public double distanceLoc2Loc(Location loc1, Location loc2) {
    double lat1 = Math.toRadians(loc1.getLatitude());
    double lat2 = Math.toRadians(loc2.getLatitude());
    double lon1 = Math.toRadians(loc1.getLongitude());
    double lon2 = Math.toRadians(loc2.getLongitude());

    double j = Math.pow(Math.sin((lat2 - lat1)/2.0), 2);
    double k = Math.pow(Math.sin((lon2 - lon1)/2.0), 2);

    return 2 * r * Math.asin(Math.sqrt( j + Math.cos(lat1) * Math.cos(lat2) * k));
  }

  @Override public double distanceLoc2Seg(Location loc, GreatCircleSegment seg) {
    return 0;
  }

  @Override public double distanceLoc2SegExtended(Location loc, GreatCircleSegment seg) {
    return 0;
  }
}
