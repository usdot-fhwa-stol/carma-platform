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
#include "Eigen/Dense"
#include <vector>
#include <lib_vehicle_model/LibVehicleModel.h>

using namespace Eigen;
using namespace std;

class UKFilter{
private:
      // State space dimension
    int n_x_;

    // Lambda which is the spreading parameter of sigma points
    double lambda_;

    // Matrix to store sigma points
    MatrixXd Xsigma_;

    // Matrix helper to calculate square root of P
    MatrixXd A_,B_;

    /*Note: Use UKF Augmentation if required for adding process noise
    //##################<UKF Augmentation>###############################

    // Augmented state space vector considering at-least 3 error co-variance as per vehicle model state space
    VectorXd x_aug_;

    // Augmented state covariance matrix considering at-least 3 error co-variance as per vehicle model state space
    MatrixXd P_aug_;

    // Augmented state space dimension
    int n_x_aug_;

     //Spreading parameter lambda after augmentation
    double lambda_aug;

    // Generate augmentation sigma point matrix
    MatrixXd Xsigma_aug_;

    //Process noise1
    double noise1_cov_;
    //Process noise2
    double noise2_cov_;
    //Process noise3
    double noise3_cov_;

//######################################################################*/
    // Predicted sigma points matrix
    MatrixXd Xsigma_pred_;
    // Weights of sigma points
    VectorXd weights_;
    //Weights
    double weight_;
    // Predicted state space vector
    VectorXd x_pred_;
    // Predicted State covariance matrix
    MatrixXd P_pred_;
    //State difference
    VectorXd x_dif;
//########################################################################
    // Measurement function variable
    VectorXd z_mf_;
    // Error comparison
    VectorXd y_err_;
    // Transpose of measurement space
    MatrixXd H_t_ ;
    MatrixXd S_ ;
    // Inverse
    MatrixXd S_i_;
    // Kalman Gain
    MatrixXd K_ ;
    //Identity Matrix
    long x_size_;
    MatrixXd I_;
    //State Update
    VectorXd X_update_;
    //Covariance Update
    MatrixXd P_update_;

    //time difference delta t between two instance
    double delta_t_;

public:

    VectorXd X(9);  //State Space Vector
    X.fill(0);      //Initialize with zero
    MatrixXd P(9,9); // State Covariance Matrix
    // Generate Identitiy matrix for the initial covariance
    P = MatrixXd::Identity(X.size(), X.size());

    enum SensorType{
        LASER,
        GPS
    } sensortype;
    // Constructor
    UKFilter();
    //Destructor

    ~UKFilter();

    void Prediction(VectorXd x, MatrixXd P,double delta_t);

    void Update(VectorXd x_prime_,MatrixXd P_prime_,MatrixXd H_,MatrixXd R_,VectorXd z_raw_);

};

#endif //V1_UKF_FILTER_H
