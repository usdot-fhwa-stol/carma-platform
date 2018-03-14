/*
 * Copyright (C) 2018 LEIDOS.
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

/**
 * This is the helper class for encoding Mobility Request message.
 * All fields' unit and type in this class match J2735 Mobility Request message.
 */
public class MobilityRequestHelper {
    
    protected static final String STRING_DEFAULT = "[]";
    protected static final int STRATEGY_MAX_LENGTH = 48;
    protected static final int PLAN_TYPE_UNKNOWN = 0;
    protected static final int URGENCY_MIN = 0;
    protected static final int URGENCY_MAX = 1000;
    protected static final int STRATEGY_PARAMS_MAX_LENGTH = 98;
    protected static final int EXPIRATION_LENGTH = Long.toString(Long.MAX_VALUE).length();

}
