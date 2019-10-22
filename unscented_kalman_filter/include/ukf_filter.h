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

        /*! \fn Prediction(VectorXd &x, MatrixXd &P,const double &delta_t_)
        \brief Angle normalization due to difference calculation between angles which can lead to small angle + 360 degree.
        \param ang The angle which needs to be normalized.
        */
        void AngleNormalization(double &ang);

    public:

        // Constructor
        //UKFilter();

        /*! \fn Prediction(VectorXd &x, MatrixXd &P,const double &delta_t_)
        \brief Prediction function uses vehicle state and its covariance from its previous state and predict its future value based on delta t.
        \param x The state space vector.
        \param P The covariance matrix of the state space.
        \param delta_t_ Time difference between previous and current state.
        */
        void Prediction(Eigen::VectorXd *x, Eigen::MatrixXd *P, double delta_t);

        /*! \fn Update(VectorXd &x_prime_, MatrixXd &P_prime_,const MatrixXd &H_,const MatrixXd &R_,const VectorXd &z_raw_)
        \brief Update function uses predicted vehicle state and its covariance compared it with the raw sensor value and do the correction based on kalman gain.
       \param x_prime_ The predicted state space vector.
       \param P_prime_ The predicted covariance matrix of the state space.
       \param H_ The measurement function.
       \param R_ The covariance matrix of the raw sensor.
       \param z_raw_ The raw sensor vector.
       */
        void
        Update(Eigen::VectorXd *x_prime, Eigen::MatrixXd *P_prime, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
               const Eigen::VectorXd &z_raw);

    };
}

#endif //V1_UKF_FILTER_H
