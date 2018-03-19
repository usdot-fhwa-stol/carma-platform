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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;

/**
 * Class to store points in a path returned by the TrajectoryConverter. 
 * Points have the following
 * 3D location either in the ECEF frame
 * A unix UTC time stamp in seconds
 */
public final class ECEFPointStamped {
  Point3D point; // Position in meters
  double stamp; // Time in seconds since Jan 1, 1970 00:00:00 UTC

  public void setPoint(Point3D point){
    this.point = point;
  }

  public void setStamp(double stamp){
    this.stamp = stamp;
  }

  public Point3D getPoint(){
    return point;
  }

  public double getStamp(){
    return stamp;
  }
}