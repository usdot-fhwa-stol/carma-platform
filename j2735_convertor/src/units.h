#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Defined the units namespace which contains many unit conversion factors
 * These are primarily meant to be used in conversions between j2735_msgs and cav_msgs
 */

namespace units {
  static const double DECA_MPS_PER_MPS = 10.0;
  static const double DECA_S_PER_S = 10.0;
  static const double MS_PER_S = 1000.0;
  static const double CM_PER_M = 100.0;
  static const double TENTH_MICRO_DEG_PER_DEG = 10000000.0;
  static const double DECA_M_PER_M = 10.0;
  static const double TWENTIETH_M_PER_M = 20.0;
  static const double FIFTIETH_M_PER_M = 50.0;
  static const double FIFTIETH_G_PER_M_PER_SEC_SQR = 5.10204081633;
  static const double ONE_AND_A_HALF_DEG = 1.5;
  static const double ONE_AND_A_HALF_DEG_PER_DEG = 0.666666666666;
  static const double CENTI_DEG_PER_DEG = 100.0;
  static const double THREE_TENTHS_DEG = 0.3;
  static const double EIGHTIETH_DEG_PER_DEG = 80.0;
  static const double DEG_360_OVER_65535_PER_DEG = 182.041666097;
  static const double UNCHANGED = 1.0;
}
