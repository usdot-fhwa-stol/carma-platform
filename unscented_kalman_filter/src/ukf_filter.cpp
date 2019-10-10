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

#include "ukf_filter.h"

namespace ukfilter {
//Constructor
    UKFilter::UKFilter() {}

    /*
    //Prediction()
    //state vector x the first argument.
    //Covariance Matrix P the second argument.
    //timestamp delta_t third argument.
    */

//##################<Prediction Cycle>#####################################
    void UKFilter::Prediction(VectorXd &x, MatrixXd &P,const double &delta_t_) {
//Basically there are three steps
//1) Generate Sigma Points with or without augmentation as required
//2) Predict Sigma Points using nine dimension vehicle model
//3) Convert the predicted sigma points into predicted mean and covariance

        // Set State space dimension
        int n_x = x.size();
        // Set lambda which is the spreading parameter of sigma points
        double lambda = (3 - n_x); //Note: The lambda value used here shows good result as per research.

//##################<Generate Sigma Points>################################
        // Generate a matrix to store sigma points
        MatrixXd Xsigma = MatrixXd(n_x, (2 * n_x + 1));

        // Finding square root of P matrix using Cholesky decomposition
        MatrixXd A = P.llt().matrixL(); //These functions in Eigen library help to find the square root of a matrix
        MatrixXd B = (sqrt(lambda + n_x)) * A;

        // First column of sigma point matrix should be mean value
        Xsigma.col(0) = x;

        // Calculating remaining sigma points
        for (int i = 0; i < n_x; i++) {
            Xsigma.col(i + 1) = x + B.col(i);
            Xsigma.col(i + n_x + 1) = x - B.col(i);
        }
        //Xsigma_ has the generated sigma point and do augmentation to add process noise.

/*Note: Use UKF Augmentation if required for adding process noise
##################<Generate Sigma Points with UKF Augmentation>###########
   // Set augmented state space vector considering at-least 3 error co-variance as per vehicle model state space
    VectorXd x_aug = VectorXd(12);

    // Create augmented state covariance matrix considering at-least 3 error co-variance as per vehicle model state space
     MatrixXd P_aug = MatrixXd(12, 12);

    // Set augmented state space dimension
    int n_x_aug = x_aug.size();
    //Set spreading parameter lambda after augmentation
    double lambda_aug=(3-n_x_aug);

    // Generate augmentation sigma point matrix
    MatrixXd Xsigma_aug = MatrixXd(n_x_aug,(( 2 * n_x_aug) + 1);

    //Process noise1
    double noise1_cov_=100;
    //Process noise2
    double noise2_cov_=200;
    //Process noise3
    double noise3_cov_=300;

    // Generate augmented state space vector [Mean value of all noise state vector is zero]
    x_aug.head(9) = x;
    x_aug(10) = 0;
    x_aug(11) = 0;
    x_aug(12) = 0;

    // Generate augmented covariance matrix with augmented noises
    P_aug.fill(0.0); //Initialize all the matrix variables with 0s
    P_aug.topLeftCorner(9,9) = P; //This function help to merge old P matrix into new from top left
    // Add noise to respective diagonal elements
    P_aug(10,10) = noise1_cov_*noise1_cov_;
    P_aug(11,11) = noise2_cov_*noise2_cov_;
    P_aug(12,12) = noise3_cov_*noise3_cov_;

    // Finding square root of augmented P matrix using Cholesky decomposition
    MatrixXd C = P_aug.llt().matrixL();
    MatrixXd D = (sqrt(lambda_aug+n_x_aug))*C;

    // First column of the augmented sigma point matrix should be mean value
    Xsigma_aug.col(0)  = x_aug;

    // Calculating remaining augmented sigma points
    for (int i = 0; i< n_x_aug; i++) {
        Xsigma_aug.col(i+1)       = x_aug + D.col(i);
        Xsigma_aug.col(i+n_x_aug+1) = x_aug - D.col(i);
    }
    //Xsigma_aug has the generated augmented sigma point

######################################################################*/

//##################<Sigma Point Prediction from Vehicle Model>#########
        //Xsigma_ ------> Vehicle Model and get back to Xsigma_pred
        // Set the predicted sigma points matrix dimensions
        MatrixXd Xsigma_pred = MatrixXd(n_x, (2 * n_x + 1));
        //Generate here
        for (int i = 0; i < 19; i++) {
            // Currently the model can only output VehicleState objects.
            //The model used can be implemented using fewer states but that is the final output.
            vector <lib_vehicle_model::VehicleState> results = lib_vehicle_model::predict(Xsigma.col(i), delta_t_,
                                                                                          delta_t_);
            Xsigma_pred.col(i, 0) = results[0].X_pos_global;
            Xsigma_pred.col(i, 1) = results[0].Y_pos_global;
            Xsigma_pred.col(i, 2) = results[0].orientation;
            Xsigma_pred.col(i, 3) = results[0].longitudinal_vel;
            Xsigma_pred.col(i, 4) = results[0].lateral_vel;
            Xsigma_pred.col(i, 5) = results[0].yaw_rate;
            Xsigma_pred.col(i, 6) = results[0].front_wheel_rotation_rate;
            Xsigma_pred.col(i, 7) = results[0].rear_wheel_rotation_rate;
            Xsigma_pred.col(i, 8) = results[0].steering_angle;
        }
        //Predicted sigma point is generated from the dynamic model.
//######################################################################

//##################<Predict Mean and Covariance from Predicted Sigma Points>#######################

        // Weights of sigma points
        VectorXd weights = VectorXd((2 * n_x) + 1);
        // Generate predicted state space vector
        VectorXd x_pred = VectorXd(n_x);
        x_pred.fill(0.0);
        // Generate state covariance matrix
        MatrixXd P_pred = MatrixXd(n_x, n_x);
        P_pred.fill(0.0);
        // Weights choosen as following based on good experimental result from research papers
        double weight = (0.5 / (n_x + lambda));
        weights.fill(weight);
        // Weights formation based on equations
        weights(0) = (lambda /(lambda + n_x)); // Weight equation of 1st column sigma point mean is different from rest

        // Extracted predicted state vector
        for (int i = 0; i < ((2 * n_x) + 1); i++) {
            x_pred = x_pred + (weights(i) * Xsigma_pred.col(i)); // x_pred is the state after time delta t
        }

        // Extracted predicted covariance matrix
        for (int i = 0; i < ((2 * n_x) + 1); i++) {
            //State difference of each sigma points
            x = Xsigma_pred.col(i) - x_pred;
            //Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
            //Yaw
            AngleNormalization(x(2));
            //Angular rotation
            AngleNormalization(x(5));
            //Steering angle
            AngleNormalization(x(8));
            // P_pred is the state after time delta t
            P = P_pred + (weights(i) * x * x.transpose());
        }

        // Prediction cycle is complete with required prior state and covariance
//###########################################################################
    }

    /*
      //Update()
      //predicted state vector x_prime_ the first argument.
      //predicted  Covariance Matrix P_prime_ the second argument.
      //measurement function the third argument.
      //Sensor Covariance R_ the fourth argument.
      //raw sensor value z_raw_ the fifth argument.
    */

//##################<Measurement Update Cycle>################################
    void UKFilter::Update(VectorXd &x_prime_, MatrixXd &P_prime_,const MatrixXd &H_,const MatrixXd &R_,const VectorXd &z_raw_) {
        // State space dimension
        int x_size = x_prime_.size();
        // Measurement function which converts the state space to measurement space
        VectorXd z_mf = H_ * x_prime_;
        // Error comparison between raw sensor value and our state belief
        VectorXd y_err = z_raw_ - z_mf;
        // Transpose of measurement space
        MatrixXd H_t = H_.transpose();
        // Kalman Gain calculations
        MatrixXd S = (H_ * P_prime_ * H_t) + R_;
        // Finding Inverse
        MatrixXd S_i = S.inverse();
        // Kalman Gain
        MatrixXd K = (P_prime_ * H_t * S_i);

        // Generate Identitiy matrix
        MatrixXd I = MatrixXd::Identity(x_size, x_size);

        // State Vector update
        x_prime_ = x_prime_ + (K * y_err);
        // Covariance Update
        P_prime_ = ((I - (K * H)) * P_prime_);

    }

//##################<Angle normalization>################################
    void UKFilter::AngleNormalization(double &ang) {
        while (ang > M_PI) ang -= 2.* M_PI;
        while (ang < -M_PI) ang += 2.* M_PI;
    }
}