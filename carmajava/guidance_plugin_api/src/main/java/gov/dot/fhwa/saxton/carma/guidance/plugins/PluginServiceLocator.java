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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.params.ParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.V2IService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;

/**
 * Service collection for the Plugin interface. Provides access to the generic, ROS agnostic interfaces
 * the plugins can use to interact with platform systems and other Guidance sub-components.
 */
public class PluginServiceLocator {
    
    private final ArbitratorService arbitratorService;
    private final PluginManagementService pluginManagementService;
    private final ParameterSource parameterSource;
    private final IPubSubService IPubSubService;
    private final ManeuverPlanner maneuverPlanner;
    private final RouteService routeService;
    private final IMobilityRouter mobilityRouter;
    private final IConflictDetector conflictDetector;
    private final ITrajectoryConverter trajectoryConverter;
    private final ILightBarManager lightBarManager;
    private final TrackingService trackingService;
    private final ITimeProvider timeProvider;
    private final V2IService v2iService;


    public PluginServiceLocator(ArbitratorService arbitratorService,
        PluginManagementService pluginManagementService, IPubSubService iPubSubService,
        ParameterSource parameterSource, ManeuverPlanner maneuverPlanner, RouteService routeService,
        IMobilityRouter mobilityRouter, IConflictDetector conflictDetector,
        ITrajectoryConverter trajectoryConverter, ILightBarManager lightBarManager, TrackingService trackingService,
        V2IService v2iService, ITimeProvider timeProvider) {
            
        this.arbitratorService = arbitratorService;
        this.IPubSubService = iPubSubService;
        this.pluginManagementService = pluginManagementService;
        this.parameterSource = parameterSource;
        this.maneuverPlanner = maneuverPlanner;
        this.routeService = routeService;
        this.mobilityRouter = mobilityRouter;
        this.conflictDetector = conflictDetector;
        this.trajectoryConverter = trajectoryConverter;
        this.lightBarManager = lightBarManager;
        this.trackingService = trackingService;
        this.timeProvider = timeProvider;
        this.v2iService = v2iService;
    }

    /**
     * Get the ArbitratorService instance available to the plugins
     */
    public ArbitratorService getArbitratorService() {
        return arbitratorService;
    }

    /**
     * Get the PluginManagementService instance available to the plugins
     */
    public PluginManagementService getPluginManagementService() {
        return pluginManagementService;
    }

    /**
     * Get the PubSubService instance available to the plugins
     */
    public IPubSubService getPubSubService() {
        return IPubSubService;
    }

    /**
     * Get the {@link ParameterSource} instance available to the plugins
     */
    public ParameterSource getParameterSource() {
        return parameterSource;
    }

    /**
     * Get the {@link ManeuverPlanner} instance available to the plugins
     */
    public ManeuverPlanner getManeuverPlanner() {
        return maneuverPlanner;
    }

    /**
     * Get the {@link RouteService} instance available to the plugins.
     */
    public RouteService getRouteService() {
        return routeService;
    }

    /**
     * Get the {@link MobilityRouter} instance available to the plugins.
     */
    public IMobilityRouter getMobilityRouter() {
        return mobilityRouter;
    }
    
    /**
     * Get the {@link IConflictDetector} instance available to the plugins
     */
    public IConflictDetector getConflictDetector() {
        return conflictDetector;
    }

    /**
     * Get the {@link ITrajectoryConverter} instance available to the plugins
     */
    public ITrajectoryConverter getTrajectoryConverter() {
        return trajectoryConverter;
    }

    /**
     * Get the {@link ILightBarManager} instance available to the plugins
     */
    public ILightBarManager getLightBarManager() {
        return this.lightBarManager;
    }

    /**
     * Get the {@link TrackingService} instance available to the plugins
     */
    public TrackingService getTrackingService() {
        return trackingService;
    }

    /**
     * Get the {@link ITimeProvider} instance available to the plugins
     */
    public ITimeProvider getTimeProvider() {
        return timeProvider;
    }
    
    /**
     * Get the {@link V2IService} instance available to the plugins
     */
    public V2IService getV2IService() {
        return v2iService;
    }
}
