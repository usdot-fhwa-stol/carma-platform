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

//Constructor
UKFilter::UKFilter() {}
//Destructor
UKFilter::~UKFilter() {}

//##################<Prediction Cycle>#####################################
void UKFilter::Prediction(VectorXd &x, MatrixXd &P,double delta_t)
{
//UKFilter need the argument as state vector x, Covariance Matrix P and timestamp delta_t
//Basically there are three steps
//1) Generate Sigma Points with or without augmentation
//2) Predict Sigma Points using a vehicle model
//3) Convert back the sigma points into predicted mean and covariance

    // Set State space dimension
    n_x_ = x.size();
   // Set lambda which is the spreading parameter of sigma points
    lambda_ = (3 - n_x_); //Note: The lambda value used here shows good result as per research.

//##################<Generate Sigma Points>################################
    // Generate a matrix to store sigma points
    Xsigma_ = MatrixXd(n_x_, (2 * n_x_ + 1));

    // Finding square root of P matrix using Cholesky decomposition
    A_= P.llt().matrixL(); //These functions in Eigen library help to find the square root of a matrix
    B_= (sqrt(lambda_+n_x_))*A_;

    // First column of sigma point matrix should be mean value
    Xsigma_.col(0) = x;

    // Calculating remaining sigma points
    for (int i = 0; i < n_x_; i++) {
        Xsigma_.col(i+1)     = x + B_.col(i);
        Xsigma_.col(i+n_x_+1) = x - B_.col(i);
    }
    //Xsigma_ has the generated sigma point and do augmentation to add process noise.

/*Note: Use UKF Augmentation if required for adding process noise
##################<Generate Sigma Points with UKF Augmentation>###########
   // Set augmented state space vector considering at-least 3 error co-variance as per vehicle model state space
    x_aug_ = VectorXd(12);

    // Create augmented state covariance matrix considering at-least 3 error co-variance as per vehicle model state space
    P_aug_ = MatrixXd(12, 12);

    // Set augmented state space dimension
    n_x_aug_ = x_aug_.size();
    //Set spreading parameter lambda after augmentation
    lambda_aug=(3-n_x_aug_);

    // Generate augmentation sigma point matrix
    Xsigma_aug_ = MatrixXd(n_x_aug_,(( 2 * n_x_aug_) + 1);

    //Process noise1
    noise1_cov_=100;
    //Process noise2
    noise2_cov_=200;
    //Process noise3
    noise3_cov_=300;

    // Generate augmented state space vector [Mean value of all noise state vector is zero]
    x_aug_.head(9) = x;
    x_aug_(10) = 0;
    x_aug_(11) = 0;
    x_aug_(12) = 0;

    // Generate augmented covariance matrix with augmented noises
    P_aug_.fill(0.0); //Initialize all the matrix variables with 0s
    P_aug_.topLeftCorner(9,9) = P; //This function help to merge old P matrix into new from top left
    // Add noise to respective diagonal elements
    P_aug_(10,10) = noise1_cov_*noise1_cov_;
    P_aug_(11,11) = noise2_cov_*noise2_cov_;
    P_aug_(12,12) = noise3_cov_*noise3_cov_;

    // Finding square root of augmented P matrix using Cholesky decomposition
    MatrixXd C = P_aug_.llt().matrixL();
    MatrixXd D = (sqrt(lambda_aug+n_x_aug_))*C;

    // First column of the augmented sigma point matrix should be mean value
    Xsigma_aug_.col(0)  = x_aug_;

    // Calculating remaining augmented sigma points
    for (int i = 0; i< n_x_aug_; i++) {
        Xsigma_aug_.col(i+1)       = x_aug_ + D.col(i);
        Xsigma_aug_.col(i+n_x_aug_+1) = x_aug_ - D.col(i);
    }
    //Xsigma_aug_ has the generated augmented sigma point

######################################################################*/

//##################<Sigma Point Prediction from Vehicle Model>#########
    //Xsigma_ ------> Vehicle Model and get back to Xsigma_pred_
    // Set the predicted sigma points matrix dimensions
    Xsigma_pred_ = MatrixXd(n_x_, (2 * n_x_ + 1));
    //Generate here
    vector<lib_vehicle_model::VehicleState> results = lib_vehicle_model::predict(state, 0.1, 0.2);
//######################################################################

//##################<Predict Mean and Covariance>#######################

    // Weights of sigma points
    weights_ = VectorXd(2 * n_x_ + 1);
    // Generate predicted state space vector
    x_pred_ = VectorXd(n_x_);
    x_pred_.fill(0.0);
    // Generate state covariance matrix
    P_pred_ = MatrixXd(n_x_, n_x_);
    P_pred_.fill(0.0);

    // Weights formation based on equations
    weight_0_ = (lambda_/(lambda_+n_x_)); // Weight equation of 1st column sigma point mean is different from rest
    weights_(0) = weight_0_;

    // Weights for rest sigma points
    for (int i=1; i<(2*n_x_+1); i++) {
        weight_ = 0.5/(n_x_+lambda_);
        weights_(i) = weight_;
    }

    // Extracted predicted state vector
    for (int i = 0; i < (2*n_x_+1); i++) {
        x_pred_ = x_pred_ + (weights_(i) * Xsigma_pred_.col(i));
    }

    // Extracted predicted covariance matrix
    for (int i = 0; i < (2*n_x_+1); i++) {
        VectorXd x_dif = Xsigma_pred_.col(i) - x;
        //Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
        //Yaw
        //    while (x_diff(2)> M_PI) x_diff(2)-=2.*M_PI;
        //    while (x_diff(2)<-M_PI) x_diff(2)+=2.*M_PI;
        //Angular rotation
        //    while (x_diff(5)> M_PI) x_diff(5)-=2.*M_PI;
        //    while (x_diff(5)<-M_PI) x_diff(5)+=2.*M_PI;
        //Steering angle
        //    while (x_diff(8)> M_PI) x_diff(8)-=2.*M_PI;
        //    while (x_diff(8)<-M_PI) x_diff(8)+=2.*M_PI;

        P_pred_ = P_pred_ + (weights_(i) * x_dif * x_dif.transpose());
    }
    //Referencing it back
    x=x_pred_;
    P= P_pred_;
    // Prediction cycle is complete with required prior state and covariance
//###########################################################################
}



//##################<Measurement Update Cycle>################################
void UKFilter::Update(VectorXd &x_prime_,MatrixXd &P_prime_,MatrixXd H_,MatrixXd R_,VectorXd z_raw_)
{
    // State space dimension
    x_size_ = x_prime_.size();
    // Measurement function which converts the state space to measurement space
    z_mf_ = H_ * x_prime_;
    // Error comparison between raw sensor value and our state belief
    y_err_ = z_raw_ - z_mf_;
    // Transpose of measurement space
    H_t_ = H_.transpose();
    // Kalman Gain calculations
    S_ = (H_ * P_prime_ * H_t_) + R_;
    // Finding Inverse
    S_i_ = S_.inverse();
    // Kalman Gain
    K_ = (P_prime_ * H_t_* S_i_);

    // Generate Identitiy matrix
    I_ = MatrixXd::Identity(x_size_, x_size_);

    // State Vector update
    X_update_ = x_prime_ + (K_ * y_err_);
    // Covariance Update
    P_update_ = ((I_ - (K_ * H_)) * P_prime_);

    //Referencing it back
    x_prime_=X_update;
    P_prime_=P_update_;
}