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

package gov.dot.fhwa.saxton.carma.message.helper;

import cav_msgs.MobilityRequest;

/**
 * This is the helper class for encoding Mobility Request message.
 * All fields' unit and type in this class match J2735 Mobility Request message.
 */
public class MobilityRequestHelper {
    
    protected static final int STRATEGY_MAX_LENGTH = 50;
    protected static final int URGENCY_MIN = 0;
    protected static final int URGENCY_MAX = 1000;
    protected static final int STRATEGY_PARAMS_MAX_LENGTH = 100;

    private MobilityHeaderHelper headerHelper;
    private byte[] strategy;
    private int planType;
    private int urgency;
    private MobilityECEFLocationHelper locationHelper;
    private byte[] strategyParams;
    private MobilityTrajectoryHelper trajectoryHelper;
    private byte[] expiration;
    
    public MobilityRequestHelper(MobilityRequest request) {
        this.headerHelper = new MobilityHeaderHelper(request.getHeader());
        this.strategy = StringConverterHelper.setDynamicLengthString(request.getStrategy(), STRATEGY_MAX_LENGTH);
        this.planType = request.getPlanType().getType();
        this.urgency = Math.min(Math.max(URGENCY_MIN, request.getUrgency()), URGENCY_MAX);
        this.locationHelper = new MobilityECEFLocationHelper(request.getLocation());
        this.strategyParams = StringConverterHelper.setDynamicLengthString(request.getStrategyParams(), STRATEGY_PARAMS_MAX_LENGTH);
        this.trajectoryHelper = new MobilityTrajectoryHelper(request.getTrajectory());
        this.expiration = StringConverterHelper.setTimestamp(request.getExpiration());
    }

    public MobilityHeaderHelper getHeaderHelper() {
        return headerHelper;
    }

    public byte[] getStrategy() {
        return strategy;
    }

    public int getPlanType() {
        return planType;
    }

    public int getUrgency() {
        return urgency;
    }

    public MobilityECEFLocationHelper getLocationHelper() {
        return locationHelper;
    }

    public byte[] getStrategyParams() {
        return strategyParams;
    }

    public MobilityTrajectoryHelper getTrajectoryHelper() {
        return trajectoryHelper;
    }

    public byte[] getExpiration() {
        return expiration;
    }

}
