/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.QuaternionUtils;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 * Class responsible for converting between the different special representations in the geometry sub-package.
 * Initially, it will only support conversion between a WGS-84 lat/lon/alt and a cartesian point.
 * cartesian points are referenced from a provided frame defined with a transform to the Earth Centered Earth Fixed Coordinate Frame (ECEF).
 * See the Geometry design document for the calculations used
 */
public class GeodesicCartesianConverter {

  protected final double Rea = 6378137.0; // Semi-major axis radius meters
  protected final double Rea_sqr = Rea*Rea;
  protected final double f = 1.0 / 298.257223563; //The flattening factor
  protected final double Reb = Rea * (1.0 - f); // //The semi-minor axis = 6356752.0
  protected final double Reb_sqr = Reb*Reb;
  protected final double e = 0.08181919084262149; // The first eccentricity (hard coded as optimization) calculated as Math.sqrt(Rea*Rea - Reb*Reb) / Rea;
  protected final double e_sqr = e*e;
  protected final double e_p = 0.08209443794969568; // e prime (hard coded as optimization) calculated as Math.sqrt((Rea_sqr - Reb_sqr) / Reb_sqr);

  /**
   * Converts a given 3d cartesian point into a WSG-84 geodesic location
   * The provided point should be defined relative to a frame which has a transform with the ECEF
   * @param point The cartesian point to be converted
   * @param ecef2frameTransform The transform which defines the position of the desired frame relative to the ECEF frame.
   * @return The calculated WSG-84 geodesic location
   */
  public Location cartesian2Geodesic(Point3D point, Transform ecef2frameTransform) {
    // ecef2frameTransform need to define the position of the desired frame relative to the ecefFrame
    // Transform into ECEF frame
    Vector3 pointBeforeTransform = new Vector3(point.getX(), point.getY(), point.getZ());
    Vector3 pointInECEF = ecef2frameTransform.apply(pointBeforeTransform);

    double x = pointInECEF.getX();
    double y = pointInECEF.getY();
    double z = pointInECEF.getZ();

    // Calculate lat,lon,alt
    double p = Math.sqrt((x*x) + (y*y));
    // Handle special case of poles
    if (p < 1.0e-10) {
      double poleLat = z < 0 ? -90:90;
      double poleLon = 0;
      double poleAlt = z < 0 ? -z - Reb : z - Reb;
      return new Location(poleLat, poleLon, poleAlt);
    }
    double theta = Math.atan((z*Rea) / (p*Reb));

    double lon = 2.0*Math.atan(y / (x + p));
    double lat = Math.atan((z + (e_p * e_p) * Reb * Math.pow(Math.sin(theta), 3)) / (p - e_sqr * Rea * Math.pow(Math.cos(theta), 3)));

    double cosLat = Math.cos(lat);
    double sinLat = Math.sin(lat);

    double N = Rea_sqr / Math.sqrt(Rea_sqr * cosLat * cosLat + Reb_sqr * sinLat * sinLat);
    double alt = (p / cosLat) - N;

    return new Location(Math.toDegrees(lat),Math.toDegrees(lon),alt);
  }

  /**
   * Converts a given WSG-84 geodesic location into a 3d cartesian point
   * The returned 3d point is defined relative to a frame which has a transform with the ECEF frame.
   * @param location The geodesic location to convert
   * @param frame2ecefTransform A transform which defines the location of the ECEF frame relative to the 3d point's frame of origin
   * @return The calculated 3d point
   */
  public Point3D geodesic2Cartesian(Location location, Transform frame2ecefTransform) {
    // frame2ecefTransform needs to define the position of the ecefFrame relative to the desired frame
    // Put geodesic in proper units
    double lonRad = location.getLonRad();
    double latRad = location.getLatRad();
    double alt = location.getAltitude();

    double sinLat = Math.sin(latRad);
    double sinLon = Math.sin(lonRad);
    double cosLat = Math.cos(latRad);
    double cosLon = Math.cos(lonRad);

    double Ne = Rea / Math.sqrt(1.0 - e_sqr * sinLat * sinLat);// The prime vertical radius of curvature

    double x = (Ne + alt)*cosLat*cosLon;
    double y = (Ne + alt)*cosLat*sinLon;
    double z = (Ne*(1-e_sqr) + alt) * sinLat;

    Vector3 pointBeforeTransform = new Vector3(x,y,z);
    Vector3 resultant = frame2ecefTransform.apply(pointBeforeTransform);

    return new Point3D(resultant.getX(), resultant.getY(), resultant.getZ());
  }

  /**
   * Calculates the transform from an ECEF frame to NED frame with it's origin at the specified location.
   * ECEF: Earth Centered Earth Fixed Frame
   * NED: North Up Down = (x,y,z)
   * @param loc The location to place the origin of the NED frame at
   * @return The calculated transform between the two frames.
   */
  public Transform ecefToNEDFromLocaton(Location loc) {
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    Point3D locInECEF =
      gcc.geodesic2Cartesian(loc, Transform.identity()); //TODO validate that this works even with an earth map transform

    Vector3 trans = new Vector3(locInECEF.getX(), locInECEF.getY(), locInECEF.getZ());

    // Rotation matrix of north east down frame with respect to ecef
    // Found at https://en.wikipedia.org/wiki/North_east_down
    double sinLat = Math.sin(loc.getLatRad());
    double sinLon = Math.sin(loc.getLonRad());
    double cosLat = Math.cos(loc.getLatRad());
    double cosLon = Math.cos(loc.getLonRad());
    double[][] R = new double[][] {
      { -sinLat * cosLon, -sinLon,  -cosLat * cosLon },
      { -sinLat * sinLon,  cosLon,  -cosLat * sinLon },
      {           cosLat,       0,           -sinLat }
    };

    Quaternion quat = QuaternionUtils.matToQuaternion(R);
    return new Transform(trans, quat);
  }
}
