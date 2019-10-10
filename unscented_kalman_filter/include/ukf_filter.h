/*
 * Copyright (C) 2019 LEIDOS.
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

#ifndef V1_UKF_FILTER_H
#define V1_UKF_FILTER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <lib_vehicle_model/LibVehicleModel.h>

namespace ukfilter {

    class UKFilter {
    private:
        /* Skip
        //time difference delta t between two instance
        double delta_t_;

        * VectorXd X(

        9);  //State Space Vector
        X.fill(0);      //Initialize with zero
        MatrixXd P(

        9,9); // State Covariance Matrix
        // Generate Identitiy matrix for the initial covariance
        P = MatrixXd::Identity(X.size(), X.size());

        enum SensorType {
            LASER,
            GPS
        } sensortype;

        */

        //Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
        void AngleNormalization(double);
    public:

        // Constructor
        UKFilter();

        //UKFilter need the argument as state vector x, Covariance Matrix P and timestamp delta_t
        void Prediction(VectorXd &x, MatrixXd &P, double delta_t);

        //Update function arguments are predicted state vector x_prime_,predicted  Covariance Matrix P_prime_
        // Sensor Covariance R_, raw sensor value z_raw_
        void Update(VectorXd &x_prime_, MatrixXd &P_prime_, const MatrixXd &H_,const MatrixXd &R_, const VectorXd &z_raw_);

    };

}

#endif //V1_UKF_FILTER_H
