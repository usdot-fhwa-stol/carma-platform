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

package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Interface for mobility message routing access to the Plugin instances.
 */
public interface IMobilityRouter {
    /**
     * Register a callback to be executed when a mobility request message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityRequestHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityRequestHandler(String strategy, MobilityRequestHandler handler);

    /**
     * Register a callback to be executed when a mobility ack message is received and is either directed
     * at the host vehicle or is a broadcast message.
     * 
     * @param handler The {@link MobilityResponseHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityResponseHandler(MobilityResponseHandler handler);

    /**
     * Register a callback to be executed when a mobility operation message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityOperationHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityOperationHandler(String strategy, MobilityOperationHandler handler);

    /**
     * Register a callback to be executed when a mobility path message is received and is either directed
     * at the host vehicle or is a broadcast message. This callback will be executed if the strategy argument
     * matches the end of the message's strategy field.
     * 
     * @param strategy The strategy value for which this callback is relevant
     * @param handler The {@link MobilityPathHandler} to invoke upon receiving a relevant event
     */
    public void registerMobilityPathHandler(String strategy, MobilityPathHandler handler);

    /**
     * Get the current vehicle's static mobility ID
     */
    public String getHostMobilityId();
    
    /**
     * Get the exclusive capability to disable or enable mobility router handling mobility path messages
     */
    public AtomicBoolean acquireDisableMobilityPathCapability();
    
    /**
     * Release the exclusive capability to disable/enable mobility router handling mobility path message
     */
    public void releaseDisableMobilityPathCapability(AtomicBoolean acquiredCapability);
    
}