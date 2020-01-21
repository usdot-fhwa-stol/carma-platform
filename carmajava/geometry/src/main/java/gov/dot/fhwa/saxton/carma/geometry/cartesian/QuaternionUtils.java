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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;

import org.ros.rosjava_geometry.Quaternion;

/**
 * Helper class for doing some quaternion operations not supported by rosjava
 */
public class QuaternionUtils {

  /**
   * Function converts a 3x3 pure rotation matrix into a normalized quaternion
   * 
   * @param m The 3x3 pure rotation matrix to be converted. Matrix must be in row major order
   * @return The normalized quaternion
   */
  public static Quaternion matToQuaternion(double[][] m) throws IllegalArgumentException{
    if (m.length != 3 || m[0].length != 3) {
      throw new IllegalArgumentException("Only 3x3 matrices can be converted to quaternion");
    }

    // Based off conversion method described
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.html
    double trace = m[0][0] + m[1][1] + m[2][2]; // Trace is the sum of the main diagonal in a matrix
    double qx, qy, qz, qw;

    if (trace > 0) { 
      double S = Math.sqrt(trace + 1.0) * 2; // S=4*qw 
      qw = 0.25 * S;
      qx = (m[2][1] - m[1][2]) / S;
      qy = (m[0][2] - m[2][0]) / S; 
      qz = (m[1][0] - m[0][1]) / S; 
    } else if ((m[0][0] > m[1][1])&&(m[0][0] > m[2][2])) { 
      double S = Math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx 
      qw = (m[2][1] - m[1][2]) / S;
      qx = 0.25 * S;
      qy = (m[0][1] + m[1][0]) / S; 
      qz = (m[0][2] + m[2][0]) / S; 
    } else if (m[1][1] > m[2][2]) { 
      double S = Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
      qw = (m[0][2] - m[2][0]) / S;
      qx = (m[0][1] + m[1][0]) / S; 
      qy = 0.25 * S;
      qz = (m[1][2] + m[2][1]) / S; 
    } else { 
      double S = Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
      qw = (m[1][0] - m[0][1]) / S;
      qx = (m[0][2] + m[2][0]) / S;
      qy = (m[1][2] + m[2][1]) / S;
      qz = 0.25 * S;
    }
    return new Quaternion(qx, qy, qz, qw).normalize();
  }

  /**
   * Function converts a normalized quaternion into a 3x3 rotation matrix
   * 
   * @param quat The normalized quaternion to convert
   * @return The 3x3 rotation matrix. Matrix is in row major order
   */
  public static double[][] quaternionToMat (Quaternion q) {
    // Based off conversion described here
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.html
    double x = q.getX();
    double y = q.getY();
    double z = q.getZ();
    double w = q.getW();
    double xx = x*x;
    double yy = y*y;
    double zz = z*z;
    double xy = x*y;
    double xz = x*z;
    double xw = x*w;
    double yz = y*z;
    double yw = y*w;
    double zw = z*w;

    double[][] m = new double[3][3];

    m[0][0] = 1.0 - 2.0 * (yy + zz);
    m[0][1] = 2.0 * (xy - zw);
    m[0][2] = 2.0 * (xz + yw);

    m[1][0] = 2.0 * (xy + zw);
    m[1][1] = 1.0 - 2.0 * (xx + zz);
    m[1][2] = 2.0 * (yz - xw);

    m[2][0] = 2.0 * (xz - yw);
    m[2][1] = 2.0 * (yz + xw);
    m[2][2] = 1 - 2.0 * (xx + yy);

    return m;
  }

}
