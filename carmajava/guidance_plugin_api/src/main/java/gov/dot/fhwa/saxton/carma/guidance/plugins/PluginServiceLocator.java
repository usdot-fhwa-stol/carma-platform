package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;

/**
 * Service collection for the Plugin interface. Provides access to the generic, ROS agnostic interfaces
 * the plugins can use to interact with platform systems and other Guidance sub-components.
 */
public class PluginServiceLocator {
    protected ArbitratorService arbitratorService;
    protected PluginService pluginService;
    protected IPubSubService IPubSubService;

    public PluginServiceLocator(ArbitratorService arbitratorService, PluginService pluginService,
        IPubSubService IPubSubService) {
        this.arbitratorService = arbitratorService;
        this.IPubSubService = IPubSubService;
        this.pluginService = pluginService;
    }

    public ArbitratorService getArbitratorService() {
        return arbitratorService;
    }

    public PluginService getPluginManager() {
        return pluginService;
    }

    public IPubSubService getPubSubManager() {
        return IPubSubService;
    }
}
