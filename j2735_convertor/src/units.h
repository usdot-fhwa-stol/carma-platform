#pragma once
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
