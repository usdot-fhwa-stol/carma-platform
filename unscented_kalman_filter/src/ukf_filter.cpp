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
 *
 *MIT License
 *
 *Copyright (c) 2017 Darien
 *
 *     Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *      to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *     copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *      copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *       AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include "ukf_filter.h"
using namespace std;
using namespace Eigen;

namespace ukfilter {
    //Constructor
    //UKFilter::UKFilter() {}

    /*! \fn Prediction(VectorXd &x, MatrixXd &P,const double &delta_t)
    \brief Prediction function uses vehicle state and its covariance from its previous state and predict its future value based on delta t.
    \param x The state space vector.
    \param P The covariance matrix of the state space.
    \param delta_t Time difference between previous and current state.
    */

//##################<Prediction Cycle>#####################################
    void UKFilter::Prediction(VectorXd *x, MatrixXd *P,double delta_t) {
//Basically there are three steps
//1) Generate Sigma Points with or without augmentation as required
//2) Predict Sigma Points using nine dimension vehicle model
//3) Convert the predicted sigma points into predicted mean and covariance

        // Set State space dimension
        int n_x = (*x).size();
        // Set lambda which is the spreading parameter of sigma points
        double lambda = (3 - n_x); //Note: The lambda value used here shows good result as per research.

//##################<Generate Sigma Points>################################
        // Generate a matrix to store sigma points
        MatrixXd Xsigma = MatrixXd(n_x, (2 * n_x + 1));

        // Finding square root of P matrix using Cholesky decomposition
        MatrixXd A = (*P).llt().matrixL(); //These functions in Eigen library help to find the square root of a matrix
        MatrixXd B = (sqrt(lambda + n_x)) * A;

        // First column of sigma point matrix should be mean value
        Xsigma.col(0) = (*x);

        // Calculating remaining sigma points
        for (int i = 0; i < n_x; i++) {
            Xsigma.col(i + 1) = (*x) + B.col(i);
            Xsigma.col(i + n_x + 1) = (*x) - B.col(i);
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
        for (int i = 0; i < (2 * n_x + 1); i++) {
            // Currently the model can only output VehicleState objects.
            //The model used can be implemented using fewer states but that is the final output.
            vector <lib_vehicle_model::VehicleState> results = lib_vehicle_model::predict(Xsigma.col(i), delta_t,
                                                                                          delta_t);
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
        // VectorXd x_pred = VectorXd(n_x);
        //x_pred.fill(0.0);
        // Generate state covariance matrix
        MatrixXd P_pred = MatrixXd(n_x, n_x);
        P_pred.fill(0.0);
        // Weights choosen as following based on good experimental result from research papers
        double weight = (0.5 / (n_x + lambda));
        weights.fill(weight);
        // Weights formation based on equations
        weights(0) = (lambda /(lambda + n_x)); // Weight equation of 1st column sigma point mean is different from rest
        (*x).fill(0);
        // Extracted predicted state vector
        for (int i = 0; i < ((2 * n_x) + 1); i++) {
            *x = *x + (weights(i) * Xsigma_pred.col(i)); // x_pred is the state after time delta t
        }
        (*P).fill(0);
        VectorXd x_diff;
        // Extracted predicted covariance matrix
        for (int i = 0; i < ((2 * n_x) + 1); i++) {
            //State difference of each sigma points
            x_diff = Xsigma_pred.col(i) - (*x);
            //Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
            //Yaw
            AngleNormalization(x_diff(2));
            //Angular rotation
            AngleNormalization(x_diff(5));
            //Steering angle
            AngleNormalization(x_diff(8));
            // P_pred is the state after time delta t
            *P = *P + (weights(i) * x_diff * x_diff.transpose());
        }

        // Prediction cycle is complete with required prior state and covariance
//###########################################################################
    }

    /*! \fn Update(VectorXd &x_prime, MatrixXd &P_prime,const MatrixXd &H,const MatrixXd &R,const VectorXd &z_raw)
   \brief Update function uses predicted vehicle state and its covariance compared it with the raw sensor value and do the correction based on kalman gain.
   \param x_prime The predicted state space vector.
   \param P_prime The predicted covariance matrix of the state space.
   \param H The measurement function.
   \param R The covariance matrix of the raw sensor.
   \param z_raw The raw sensor vector.
   */

//##################<Measurement Update Cycle>################################
    void UKFilter::Update(VectorXd *x_prime, MatrixXd *P_prime,const MatrixXd &H,const MatrixXd &R,const VectorXd &z_raw) {
        // State space dimension
        int x_size = (*x_prime).size();
        // Measurement function which converts the state space to measurement space
        VectorXd z_mf = H * (*x_prime);
        // Error comparison between raw sensor value and our state belief
        VectorXd y_err = z_raw - z_mf;
        // Transpose of measurement space
        MatrixXd H_t = H.transpose();
        // Kalman Gain calculations
        MatrixXd S = (H * (*P_prime) * H_t) + R;
        // Finding Inverse
        MatrixXd S_i = S.inverse();
        // Kalman Gain
        MatrixXd K = ((*P_prime) * H_t * S_i);

        // Generate Identitiy matrix
        MatrixXd I = MatrixXd::Identity(x_size, x_size);

        // State Vector update
        *x_prime = (*x_prime) + (K * y_err);
        // Covariance Update
        *P_prime = ((I - (K * H)) * (*P_prime));

    }
    /*! \fn AngleNormalization(double &ang)
   \brief Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
   \param ang The angle which needs to be normalized.
   */
//##################<Angle normalization>################################
    void UKFilter::AngleNormalization(double &ang) {
        while (ang > M_PI) ang -= 2.* M_PI;
        while (ang < -M_PI) ang += 2.* M_PI;
    }
}