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

import gov.dot.fhwa.saxton.carma.geometry.cartesian.*;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Vector3;

import static org.junit.Assert.*;

/**
 * Unit tests for the CartesianObject class
 */
public class QuaternionUtilsTest {
  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(CartesianObjectTest.class);
    log.info("Setting up tests for Vector");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the quaternionToMat function
   * @throws Exception
   */
  @Test
  public void testQuaternionToMat() throws Exception {
    // Test identity
    Quaternion quat = Quaternion.identity();
    double[][] solution = {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}
    };
    double[][] result = QuaternionUtils.quaternionToMat(quat);
    assertTrue(matEquals(solution, result, 0.00001));

    // Test x-axis
    quat = Quaternion.fromAxisAngle(new Vector3(1, 0, 0), Math.toRadians(90));
    solution = new double[][] {
      {1.0, 0.0, 0.0},
      {0.0, 0.0, -1.0},
      {0.0, 1.0, 0.0}
    };
    result = QuaternionUtils.quaternionToMat(quat);
    assertTrue(matEquals(solution, result, 0.00001));

    // Test y-axis
    quat = Quaternion.fromAxisAngle(new Vector3(0, 1, 0), Math.toRadians(90));
    solution = new double[][] {
      {0.0, 0.0, 1.0},
      {0.0, 1.0, 0.0},
      {-1.0, 0.0, 0.0}
    };
    result = QuaternionUtils.quaternionToMat(quat);
    assertTrue(matEquals(solution, result, 0.00001));
    
    // Test z-axis
    quat = Quaternion.fromAxisAngle(new Vector3(0, 0, 1), Math.toRadians(90));
    solution = new double[][] {
      {0.0, -1.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 0.0, 1.0}
    };
    result = QuaternionUtils.quaternionToMat(quat);
    assertTrue(matEquals(solution, result, 0.00001));

    // Test complex rotation
    quat = new Quaternion(0.5, 0.5, 0.0, 0.7071068);
    solution = new double[][] {
      {0.5,         0.5,       0.7071068},
      {0.5,         0.5,      -0.7071068},
      {-0.7071068,  0.7071068, 0.0}
    };
    result = QuaternionUtils.quaternionToMat(quat);
    assertTrue(matEquals(solution, result, 0.00001));
  }

  /**
   * Tests the matToQuaternion function
   * @throws Exception
   */
  @Test
  public void testMatToQuaternion() throws Exception {
    // Test identity
    Quaternion solution = Quaternion.identity();
    double[][] mat = {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0}
    };
    Quaternion result = QuaternionUtils.matToQuaternion(mat);
    assertTrue(quatEquals(solution, result, 0.00001));

    // Test x-axis
    solution = Quaternion.fromAxisAngle(new Vector3(1, 0, 0), Math.toRadians(90));
    mat = new double[][] {
      {1.0, 0.0, 0.0},
      {0.0, 0.0, -1.0},
      {0.0, 1.0, 0.0}
    };
    result = QuaternionUtils.matToQuaternion(mat);
    assertTrue(quatEquals(solution, result, 0.00001));

    // Test y-axis
    solution = Quaternion.fromAxisAngle(new Vector3(0, 1, 0), Math.toRadians(90));
    mat = new double[][] {
      {0.0, 0.0, 1.0},
      {0.0, 1.0, 0.0},
      {-1.0, 0.0, 0.0}
    };
    result = QuaternionUtils.matToQuaternion(mat);
    assertTrue(quatEquals(solution, result, 0.00001));
    
    // Test z-axis
    solution = Quaternion.fromAxisAngle(new Vector3(0, 0, 1), Math.toRadians(90));
    mat = new double[][] {
      {0.0, -1.0, 0.0},
      {1.0, 0.0, 0.0},
      {0.0, 0.0, 1.0}
    };
    result = QuaternionUtils.matToQuaternion(mat);
    assertTrue(quatEquals(solution, result, 0.00001));

    // Test complex rotation
    solution = new Quaternion(0.5, 0.5, 0.0, 0.7071068);
    mat = new double[][] {
      {0.5,         0.5,       0.7071068},
      {0.5,         0.5,      -0.7071068},
      {-0.7071068,  0.7071068, 0.0}
    };
    result = QuaternionUtils.matToQuaternion(mat);
    assertTrue(quatEquals(solution, result, 0.00001));
  }

  /**
   * Helper function to test if two 3x3 matrices are elementwise equivalent within some epsilon
   */
  private boolean matEquals(double[][] mat1, double[][] mat2, double epsilon) throws IllegalArgumentException {
    if (mat1.length != 3 || mat1[0].length != 3 || mat2.length !=3 || mat2[0].length != 3) {
      throw new IllegalArgumentException("Cannot compare non 3x3 matrix");
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (Math.abs(mat1[i][j] - mat2[i][j]) > epsilon) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Helper function to test if two quaternions are equivalent with some epsilon
   */
  private boolean quatEquals(Quaternion q1, Quaternion q2, double epsilon) {
    return Math.abs(q1.getX() - q2.getX()) < epsilon
          && Math.abs(q1.getY() - q2.getY()) < epsilon
          && Math.abs(q1.getZ() - q2.getZ()) < epsilon
          && Math.abs(q1.getW() - q2.getW()) < epsilon;
  }

}
