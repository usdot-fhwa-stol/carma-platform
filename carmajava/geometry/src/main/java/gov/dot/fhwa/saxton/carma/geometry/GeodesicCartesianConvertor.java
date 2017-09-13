/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 * Class responsible for converting between the different special representations in the geometry sub-package.
 * Initially, it will only support conversion between a WGS-84 lat/lon/alt and a cartesian point.
 * cartesian points are referenced from a provided frame defined with a transform to the Earth Centered Earth Fixed Coordinate Frame (ECEF).
 * See the Geometry design document for the calculations used
 */
public class GeodesicCartesianConvertor {

  protected final double Rea = 6378137.0; // Semi-major axis radius meters
  protected final double Rea_sqr = Rea*Rea;
  //The flattening factor f = 1.0 / 298.257223563
  protected final double Reb = 6356752.0; // //The semi-minor axis = Rea * (1.0 - f)
  protected final double Reb_sqr = Reb*Reb;
  protected final double e = 0.08181919; // The first eccentricity Math.sqrt(Rea*Rea - Reb*Reb) / Rea;
  protected final double e_sqr = e*e;

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
      double poleAlt = z < 0 ? z + Reb : z - Reb;
      return new Location(poleLat, poleLon, poleAlt);
    }
    double theta = Math.atan((z*Rea) / (p*Reb));
    double e_p = Math.sqrt((Rea_sqr - Reb_sqr) / Reb_sqr);

    double lon = 2.0*Math.atan(y / (x + Math.sqrt((x*x) + (y*y))));
    double lat = Math.atan((z + (e_p * e_p) * Reb * Math.pow(Math.sin(theta), 3)) / (p - e_sqr * Rea * Math.pow(Math.cos(theta), 3)));

    double N = Rea_sqr / Math.sqrt(Rea_sqr * Math.pow(Math.cos(lat), 2) + Reb_sqr * Math.pow(Math.sin(lat),2));
    double alt = (p / Math.cos(lat)) - N;

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

    double Ne = Rea / Math.sqrt(1.0 - e_sqr * Math.pow(Math.sin(latRad),2));// The prime vertical radius of curvature

    double x = (Ne + alt)*Math.cos(latRad)*Math.cos(lonRad);
    double y = (Ne + alt)*Math.cos(latRad)*Math.sin(lonRad);
    double z = (Ne*(1-e_sqr) + alt) * Math.sin(latRad);

    Vector3 pointBeforeTransform = new Vector3(x,y,z);
    Vector3 resultant = frame2ecefTransform.apply(pointBeforeTransform);

    return new Point3D(resultant.getX(), resultant.getY(), resultant.getZ());
  }
}
