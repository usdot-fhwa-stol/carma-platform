/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#ifndef PREDICT_CTRV_H
#define PREDICT_CTRV_H

#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/predicted_state.hpp>

namespace motion_predict
{
namespace ctrv
{
/*!
 * \brief Generates a set of motion predictions seperated by the given time step size for the given period.
 *        Predictions are made using a CTRV motion model.
 *
 * \param  obj external object to predict
 * \param  delta_t prediction time step size in seconds
 * \param  period The total prediction period in seconds
 * \param  process_noise_max is the maximum process noise of the system
 *
 * \return The predicted state of the external object at time t + delta_t
 */
std::vector<carma_perception_msgs::msg::PredictedState> predictPeriod(const carma_perception_msgs::msg::ExternalObject& obj, const double delta_t,
                                                    const double period, const float process_noise_max,
                                                    const double confidence_drop_rate);
/*!
 * \brief predictStep populates motion prediction with future pose and velocity.
 *     The predicted motion is created using a CTRV motion model
 *
 * \param  obj external object.
 * \param  delta_t prediction time into the future in seconds
 * \param  process_noise_max is the maximum process noise of the system
 *
 * \return The predicted state of the external object at time t + delta_t
 */

carma_perception_msgs::msg::PredictedState predictStep(const carma_perception_msgs::msg::ExternalObject& obj, const double delta_t,
                                     const float process_noise_max);

/*!
 * \brief predictStep populates motion prediction with future pose and velocity.
 *     The predicted motion is created using a CTRV motion model.
 *
 * \param  obj previous prediction object.
 * \param  delta_t prediction time into the future in seconds
 * \param  process_noise_max is the maximum process noise of the system
 *
 * \return The predicted state of the external object at time t + delta_t
 */

carma_perception_msgs::msg::PredictedState predictStep(const carma_perception_msgs::msg::PredictedState& obj, const double delta_t,
                                     const double confidence_drop_rate);

}  // namespace ctrv

}  // namespace predict

#endif /* PREDICT_CTRV_H */