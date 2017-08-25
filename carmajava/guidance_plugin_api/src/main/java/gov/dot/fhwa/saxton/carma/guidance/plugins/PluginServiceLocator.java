package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;

/**
 * Service collection for the Plugin interface. Provides access to the generic, ROS agnostic interfaces
 * the plugins can use to interact with platform systems and other Guidance sub-components.
 */
public class PluginServiceLocator {
    protected ArbitratorService arbitratorService;
    protected PluginManagementService pluginManagementService;
    protected IPubSubService IPubSubService;

    public PluginServiceLocator(ArbitratorService arbitratorService,
        PluginManagementService pluginManagementService, IPubSubService IPubSubService) {
        this.arbitratorService = arbitratorService;
        this.IPubSubService = IPubSubService;
        this.pluginManagementService = pluginManagementService;
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
}
