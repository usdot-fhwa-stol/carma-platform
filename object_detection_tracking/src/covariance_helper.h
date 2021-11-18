#pragma once

// Copyright 2008 Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * 
 * Modifications (c) Leidos 2021
 * - Moved to new namespace.
 * 
 */ 


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace covariance_helper {

/** 
 * NOTE: The function is a direct copy from 
 * https://github.com/ros2/geometry2/blob/16562cee8694c11f1f82b2bdbde2814fca1c7954/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.hpp#L422
 * 
 * This bit of logic is normally included in the tf2_geometry_msgs packages, but it has not been backported to ROS2 Foxy
 * This means the current doTransform operation for PoseWithCovarianceStamped does not result in the correct covariance values
 * This function should be called after a call to doTransform to get the correct transformed covariance
 * 
 * \brief Transform the covariance matrix of a PoseWithCovariance message to a new frame.
 * \param cov_in The covariance matrix to transform.
 * \param transform The transform to apply, as a tf2::Transform structure.
 * \return The transformed covariance matrix.
 */
inline
geometry_msgs::msg::PoseWithCovariance::_covariance_type transformCovariance(
  const geometry_msgs::msg::PoseWithCovariance::_covariance_type & cov_in,
  const tf2::Transform & transform)
{
  /**
   * To transform a covariance matrix:
   *
   * \verbatim[R 0] COVARIANCE [R' 0 ]
   [0 R]            [0  R']\endverbatim
   *
   * Where:
   *         R is the rotation matrix (3x3).
   *         R' is the transpose of the rotation matrix.
   *         COVARIANCE is the 6x6 covariance matrix to be transformed.
   *
   * Reference:
   *         A. L. Garcia, “Linear Transformations of Random Vectors,” in Probability,
   *         Statistics, and Random Processes For Electrical Engineering, 3rd ed.,
   *         Pearson Prentice Hall, 2008, pp. 320–322.
   */

  // get rotation matrix (and transpose)
  const tf2::Matrix3x3 R = transform.getBasis();
  const tf2::Matrix3x3 R_transpose = R.transpose();

  // convert covariance matrix into four 3x3 blocks
  // *INDENT-OFF*
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2],
                              cov_in[6], cov_in[7], cov_in[8],
                              cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5],
                              cov_in[9], cov_in[10], cov_in[11],
                              cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20],
                              cov_in[24], cov_in[25], cov_in[26],
                              cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23],
                              cov_in[27], cov_in[28], cov_in[29],
                              cov_in[33], cov_in[34], cov_in[35]);
  // *INDENT-ON*

  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = R * cov_11 * R_transpose;
  const tf2::Matrix3x3 result_12 = R * cov_12 * R_transpose;
  const tf2::Matrix3x3 result_21 = R * cov_21 * R_transpose;
  const tf2::Matrix3x3 result_22 = R * cov_22 * R_transpose;

  // form the output
  geometry_msgs::msg::PoseWithCovariance::_covariance_type cov_out;
  cov_out[0] = result_11[0][0];
  cov_out[1] = result_11[0][1];
  cov_out[2] = result_11[0][2];
  cov_out[6] = result_11[1][0];
  cov_out[7] = result_11[1][1];
  cov_out[8] = result_11[1][2];
  cov_out[12] = result_11[2][0];
  cov_out[13] = result_11[2][1];
  cov_out[14] = result_11[2][2];

  cov_out[3] = result_12[0][0];
  cov_out[4] = result_12[0][1];
  cov_out[5] = result_12[0][2];
  cov_out[9] = result_12[1][0];
  cov_out[10] = result_12[1][1];
  cov_out[11] = result_12[1][2];
  cov_out[15] = result_12[2][0];
  cov_out[16] = result_12[2][1];
  cov_out[17] = result_12[2][2];

  cov_out[18] = result_21[0][0];
  cov_out[19] = result_21[0][1];
  cov_out[20] = result_21[0][2];
  cov_out[24] = result_21[1][0];
  cov_out[25] = result_21[1][1];
  cov_out[26] = result_21[1][2];
  cov_out[30] = result_21[2][0];
  cov_out[31] = result_21[2][1];
  cov_out[32] = result_21[2][2];

  cov_out[21] = result_22[0][0];
  cov_out[22] = result_22[0][1];
  cov_out[23] = result_22[0][2];
  cov_out[27] = result_22[1][0];
  cov_out[28] = result_22[1][1];
  cov_out[29] = result_22[1][2];
  cov_out[33] = result_22[2][0];
  cov_out[34] = result_22[2][1];
  cov_out[35] = result_22[2][2];

  return cov_out;
}

} // covariance_helper