/*
 * Copyright (C) 2018-2019 LEIDOS.
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

import cav_msgs.Plugin;
import cav_msgs.SystemAlert;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceAction;
import gov.dot.fhwa.saxton.carma.guidance.params.RosParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginLifecycleHandler.PluginState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.IStateChangeListener;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.TrackingService;
import gov.dot.fhwa.saxton.carma.guidance.conflictdetector.IConflictDetector;
import gov.dot.fhwa.saxton.carma.guidance.lightbar.ILightBarManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.mobilityrouter.IMobilityRouter;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.util.ITimeProvider;
import gov.dot.fhwa.saxton.carma.guidance.util.RouteService;
import gov.dot.fhwa.saxton.carma.guidance.util.V2IDataCallback;
import gov.dot.fhwa.saxton.carma.guidance.util.V2IService;
import gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter.ITrajectoryConverter;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

import org.reflections.Reflections;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceException;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import std_msgs.Header;

import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Guidance package PluginManager component
 * <p>
 * Responsible for instantiating, running, owning, and runtime management of
 * all plugins installed in the software's operating environment.
 */
public class PluginManager extends GuidanceComponent implements AvailabilityListener, IStateChangeListener, PluginManagementService {
    protected final long sleepDurationMillis = 30000;
    protected int sequenceNumber = 0;

    protected PluginExecutor executor;
    protected PluginServiceLocator pluginServiceLocator;
    protected List<IPlugin> registeredPlugins = new ArrayList<>();
    protected List<String> ignoredPluginClassNames = new ArrayList<>();
    protected List<String> requiredPluginClassNames = new ArrayList<>();

    protected final String PLUGIN_DISCOVERY_ROOT = "gov.dot.fhwa.saxton.carma";
    protected final String messagingBaseUrl = "plugins";
    protected final String getRegisteredPluginsServiceUrl = "get_registered_plugins";
    protected final String getActivePluginsServiceUrl = "get_active_plugins";
    protected final String activatePluginServiceUrl = "activate_plugin";
    protected final String availablePluginsTopicUrl = "available_plugins";

    protected ServiceServer<PluginListRequest, PluginListResponse> registeredPluginService;
    protected ServiceServer<PluginListRequest, PluginListResponse> activePluginService;
    protected ServiceServer<PluginActivationRequest, PluginActivationResponse> activatePluginService;
    protected Publisher<cav_msgs.PluginList> pluginPublisher;

    protected MessageFactory messageFactory;

    protected int availablePluginsSeqNum = 0;
    protected int registeredPluginsSeqNum = 0;
    protected int activePluginsSeqNum = 0;

    public PluginManager(GuidanceStateMachine stateMachine, IPubSubService pubSubManager, 
    IGuidanceCommands commands, IManeuverInputs maneuverInputs, RouteService routeService,
    ConnectedNode node, IMobilityRouter router, IConflictDetector conflictDetector,
    ITrajectoryConverter trajectoryConverter, ILightBarManager lightBarManager,
    TrackingService trackingService, V2IService v2iService, ITimeProvider timeProvider) {
        super(stateMachine, pubSubManager, node);
        this.executor = new PluginExecutor();

        pluginServiceLocator = new PluginServiceLocator(
                null, // Need to call setArbitrator service to resolve circular dependency
                this,
                pubSubService, 
                new RosParameterSource(node.getParameterTree()), 
                new ManeuverPlanner(commands, maneuverInputs), 
                routeService, router , conflictDetector, trajectoryConverter,
                lightBarManager, trackingService, v2iService, timeProvider);
    }

    /**
     * Set the arbitrator service available to the plugins
     * <p>
     * Used to resolve circular dependency in constructors
     */
    public void setArbitratorService(ArbitratorService arbitratorService) {
        pluginServiceLocator = new PluginServiceLocator(
                arbitratorService,
                pluginServiceLocator.getPluginManagementService(),
                pluginServiceLocator.getPubSubService(), 
                pluginServiceLocator.getParameterSource(), 
                pluginServiceLocator.getManeuverPlanner(), 
                pluginServiceLocator.getRouteService(),
                pluginServiceLocator.getMobilityRouter(),
                pluginServiceLocator.getConflictDetector(),
                pluginServiceLocator.getTrajectoryConverter(),
                pluginServiceLocator.getLightBarManager(),
                pluginServiceLocator.getTrackingService(),
                pluginServiceLocator.getV2IService(),
                pluginServiceLocator.getTimeProvider());
        jobQueue.add(this::onStartup);
        stateMachine.registerStateChangeListener(this);
    }

    /**
     * Detect all IPlugin instances available on classpath at or below PLUGIN_DISCOVERY_ROOT
     * @return
     */
    protected List<Class<? extends IPlugin>> discoverPluginsOnClasspath() {
        Reflections pluginDiscoverer = new Reflections(PLUGIN_DISCOVERY_ROOT);
        Set<Class<? extends IPlugin>> pluginClasses = pluginDiscoverer.getSubTypesOf(IPlugin.class);

        List<Class<? extends IPlugin>> out = new ArrayList<>();
        for (Class<? extends IPlugin> pluginClass : pluginClasses) {
            boolean ignored = false;
            for (String ignoredPluginName : ignoredPluginClassNames) {
                // Filter the names against the list of ignored plugins
                if (pluginClass.getName().equals(ignoredPluginName)) {
                    ignored = true;
                }
            }

            // Check if we haven't ignored it and that it isn't an abstract class or an interface
            if (!ignored && !Modifier.isAbstract(pluginClass.getModifiers())
                    && !Modifier.isInterface(pluginClass.getModifiers())) {

                out.add(pluginClass);
                if (log != null) {
                    log.info("PLUGIN", "Guidance.PluginManager will initialize plugin: " + pluginClass.getName());
                }
            } else {
                if (log != null) {
                    log.info("PLUGIN", "Guidance.PluginManager will ignore plugin: " + pluginClass.getName());
                }
            }
        }

        return out;
    }

    /**
     * Instantiate a list of plugins classes into live objects
     * @param classes The list of classes which implement IPlugin
     * @param pluginServiceLocator The service locator to pass as their constructor argument
     * @return A list containing instantiated plugin instances where the instantiation was successful
     */
    protected List<IPlugin> instantiatePluginsFromClasses(List<Class<? extends IPlugin>> classes,
            PluginServiceLocator pluginServiceLocator) {
        List<IPlugin> pluginInstances = new ArrayList<>();
        for (Class<? extends IPlugin> pluginClass : classes) {
            try {
                Constructor<? extends IPlugin> pluginCtor = pluginClass.getConstructor(PluginServiceLocator.class);

                // TODO: This is brittle, depends on convention of having a constructor taking only a PSL
                IPlugin pluginInstance = pluginCtor.newInstance(pluginServiceLocator);
                pluginInstance.registerAvailabilityListener(this);
                ComponentVersion version = pluginInstance.getVersionInfo();
                log.info("PLUGIN", "Guidance.PluginManager instantiated new plugin instance: " + version.componentName() + ":"
                        + version.revisionString());

                // If the plugin is required activate it by default
                if (requiredPluginClassNames.contains(pluginInstance.getClass().getName())) {
                    pluginInstance.setActivation(true);
                }

                pluginInstances.add(pluginInstance);
            } catch (Exception e) {
                log.error("PLUGIN", "Unable to instantiate: " + pluginClass.getCanonicalName(), e);
            }
        }

        return pluginInstances;
    }

    public List<IPlugin> getRegisteredPlugins() {
        return registeredPlugins;
    }

    @Override
    public String getComponentName() {
        return "Guidance.PluginManager";
    }

    @Override
    public void onStartup() {
        // Instantiate the plugins and register them
        ignoredPluginClassNames = (List<String>) node.getParameterTree().getList("~ignored_plugins", new ArrayList<>());
        requiredPluginClassNames = (List<String>) node.getParameterTree().getList("~required_plugins", new ArrayList<>());

        log.info("STARTUP", "Ignoring plugins: " + ignoredPluginClassNames.toString());
        log.info("STARTUP", "Requiring plugins: " + requiredPluginClassNames.toString());
        List<Class<? extends IPlugin>> pluginClasses = discoverPluginsOnClasspath();

        registeredPlugins = instantiatePluginsFromClasses(pluginClasses, pluginServiceLocator);
        for (IPlugin p : getRegisteredPlugins()) {
            ComponentVersion v = p.getVersionInfo();
            executor.submitPlugin(p);
            executor.initializePlugin(v.componentName(), v.revisionString()); // could provide all info in one arg
                                                                              // with v.toString()

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            	log.error("STARTUP", e.getMessage());
            }
        }

        // Setup the services related to plugin queries
        setupServices();

        // Configure the plugin availability topic and topic message factory
        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
        messageFactory = nodeConfig.getTopicMessageFactory();
        pluginPublisher = node.newPublisher(messagingBaseUrl + "/" + availablePluginsTopicUrl,
                cav_msgs.PluginList._TYPE);
        pluginPublisher.setLatchMode(true);
        
        currentState.set(GuidanceState.STARTUP);
    }

    @Override
    public void onSystemReady() {
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    @Override
    public void onActive() {
        currentState.set(GuidanceState.ACTIVE);

        for (IPlugin p : getRegisteredPlugins()) {
            ComponentVersion v = p.getVersionInfo();
            if (p.getActivation()) {
                executor.resumePlugin(v.componentName(), v.revisionString());
            }
        }
    }
    
    @Override
    public void onDeactivate() {
        currentState.set(GuidanceState.INACTIVE);
    }
    
    @Override
    public void onEngaged() {
        currentState.set(GuidanceState.ENGAGED);
    }

    @Override
    public void onCleanRestart() {
        shutdownPlugins();
        
        List<Class<? extends IPlugin>> pluginClasses = discoverPluginsOnClasspath();

        registeredPlugins = instantiatePluginsFromClasses(pluginClasses, pluginServiceLocator);
        for (IPlugin p : getRegisteredPlugins()) {
            ComponentVersion v = p.getVersionInfo();
            executor.submitPlugin(p);
            executor.initializePlugin(v.componentName(), v.revisionString());
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            	log.error("RESTART", e.getMessage());
            }
        }
        
        currentState.set(GuidanceState.DRIVERS_READY);
    }

    /**
     * Cleanly shutdown all plugins that were started
     */
    private void shutdownPlugins() {
        // If we're shutting down, properly handle graceful plugin shutdown as well
        for (IPlugin p : getRegisteredPlugins()) {
            PluginState pState = executor.getPluginState(p.getVersionInfo().componentName(), p.getVersionInfo().revisionString());
            if (pState == PluginState.RESUMED || pState == PluginState.LOOPING) {
                ComponentVersion v = p.getVersionInfo();
                p.setActivation(false);
                executor.suspendPlugin(v.componentName(), v.revisionString());
            }
        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
        	log.error("SHUTDOWN", e.getMessage());
        }

        for (IPlugin p : getRegisteredPlugins()) {
            PluginState pState = executor.getPluginState(p.getVersionInfo().componentName(), p.getVersionInfo().revisionString());
            if (pState != PluginState.UNINITIALIZED) {
                ComponentVersion v = p.getVersionInfo();
                executor.terminatePlugin(v.componentName(), v.revisionString());
            }
        }
    }
    
    @Override
    public void onShutdown() {
        log.fatal(getComponentName() + " is about to SHUTDOWN!");
        
        currentState.set(GuidanceState.SHUTDOWN);

        shutdownPlugins();
        
        // Log the fatal error
        log.fatal("!!!!! Guidance component " + getComponentName() + " has entered a PANIC state !!!!!");

        // Alert the other ROS nodes to the FATAL condition
        IPublisher<SystemAlert> pub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);
        SystemAlert fatalBroadcast = pub.newMessage();
        fatalBroadcast.setDescription(getComponentName() + " has triggered a Guidance PANIC");
        fatalBroadcast.setType(SystemAlert.FATAL);
        pub.publish(fatalBroadcast);

        // Cancel the loop
        timingLoopThread.interrupt();
        loopThread.interrupt();
    }

    /**
     * Listen for availability change from the plugin instances
     * @param plugin The plugin who's availability has changed
     * @param availability The new state of that plugin's availability
     */
    @Override
    public void onAvailabilityChange(IPlugin plugin, boolean availability) {
        if (pluginPublisher == null) {
            // Bail if we can't yet publish
            return;
        }

        cav_msgs.PluginList availablePlugins = messageFactory.newFromType(cav_msgs.PluginList._TYPE);

        // Publish mock available plugin status
        Header h = messageFactory.newFromType(Header._TYPE);
        h.setStamp(node.getCurrentTime());
        h.setFrameId("0");
        h.setSeq(availablePluginsSeqNum++);
        availablePlugins.setHeader(h);

        /* Rather than maintain a separate list of plugins, which we'd have to walk anyway to update
         * just walk the main list every time one changes status. With a small number of plugins this
         * shouldn't be a performance concern
         */
        List<Plugin> pList = new ArrayList<>();
        for (IPlugin p : registeredPlugins) {
            if (p.getAvailability()) {
                Plugin pMsg = messageFactory.newFromType(Plugin._TYPE);
                ComponentVersion v = p.getVersionInfo();
                pMsg.setAvailable(p.getAvailability());
                pMsg.setName(v.componentName());
                pMsg.setVersionId(v.revisionString());
                pMsg.setActivated(p.getActivation());
                pMsg.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                pList.add(pMsg);
            }
        }

		//Sort
		pList.sort(
			(Plugin p1, Plugin p2) -> p1.getName().compareToIgnoreCase(p2.getName())
		);

        availablePlugins.setPlugins(pList);
        pluginPublisher.publish(availablePlugins);
    }

    /**
     * Configure all the services to respond with dummy data
     */
    private void setupServices() {
        registeredPluginService = node.newServiceServer(messagingBaseUrl + "/" + getRegisteredPluginsServiceUrl,
                PluginList._TYPE, new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
                    @Override
                    public void build(PluginListRequest pluginListRequest, PluginListResponse pluginListResponse)
                            throws ServiceException {
                        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
                        MessageFactory factory = nodeConfig.getTopicMessageFactory();

                        // TODO: Refactor this out to the PluginList message, not the Plugin msg
                        Header h = factory.newFromType(Header._TYPE);
                        h.setStamp(node.getCurrentTime());
                        h.setFrameId("0");
                        h.setSeq(registeredPluginsSeqNum++);
                        pluginListResponse.setHeader(h);

                        List<Plugin> pList = new ArrayList<>();
                        for (IPlugin p : registeredPlugins) {
                            Plugin p0 = factory.newFromType(Plugin._TYPE);
                            ComponentVersion v = p.getVersionInfo();
                            p0.setAvailable(p.getAvailability());
                            p0.setName(v.componentName());
                            p0.setVersionId(v.revisionString());
                            p0.setActivated(p.getActivation());
                            p0.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                            pList.add(p0);
                        }

						//Sort
						pList.sort(
						  (Plugin p1, Plugin p2) -> p1.getName().compareToIgnoreCase(p2.getName())
						);

                        pluginListResponse.setPlugins(pList);
                    }
                });

        activePluginService = node.newServiceServer(messagingBaseUrl + "/" + getActivePluginsServiceUrl,
                PluginList._TYPE, new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
                    @Override
                    public void build(PluginListRequest pluginListRequest, PluginListResponse pluginListResponse)
                            throws ServiceException {
                        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
                        MessageFactory factory = nodeConfig.getTopicMessageFactory();

                        Header h = factory.newFromType(Header._TYPE);
                        h.setStamp(node.getCurrentTime());
                        h.setFrameId("0");
                        h.setSeq(activePluginsSeqNum++);
                        pluginListResponse.setHeader(h);

                        List<Plugin> pList = new ArrayList<>();
                        // Walk the plugin list and see which ones are active
                        for (IPlugin p : registeredPlugins) {
                            if (p.getActivation()) {
                                Plugin p0 = factory.newFromType(Plugin._TYPE);
                                ComponentVersion v = p.getVersionInfo();
                                p0.setAvailable(p.getAvailability());
                                p0.setName(v.componentName());
                                p0.setVersionId(v.revisionString());
                                p0.setActivated(p.getActivation());
                                p0.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                                pList.add(p0);
                            }
                        }

						//Sort
						pList.sort(
						  (Plugin p1, Plugin p2) -> p1.getName().compareToIgnoreCase(p2.getName())
						);

                        pluginListResponse.setPlugins(pList);
                    }
                });

        activatePluginService = node.newServiceServer(messagingBaseUrl + "/" + activatePluginServiceUrl,
                PluginActivation._TYPE,
                new ServiceResponseBuilder<PluginActivationRequest, PluginActivationResponse>() {
                    @Override
                    public void build(PluginActivationRequest pluginActivationRequest,
                            PluginActivationResponse pluginActivationResponse) throws ServiceException {
                        // Walk the plugin list and see which one matches the name and version
                        for (IPlugin p : registeredPlugins) {
                        	ComponentVersion v = p.getVersionInfo();
                            if (pluginActivationRequest.getPluginName().equals(v.componentName())
                                    && pluginActivationRequest.getPluginVersion().equals(v.revisionString())) {
                                // Match detected
                                if (!requiredPluginClassNames.contains(p.getClass().getName())) {
                                    // Can't change state of required plugins
                                    p.setActivation(pluginActivationRequest.getActivated());
                                }
                                pluginActivationResponse.setNewState(p.getActivation());
                                return;
                            }
                        }

                        pluginActivationResponse.setNewState(false);
                    }
                });
    }

    @Override
    public void onStateChange(GuidanceAction action) {
        log.debug("GUIDANCE_STATE", getComponentName() + " received action: " + action);
        switch (action) {
        case INTIALIZE:
            jobQueue.add(this::onSystemReady);
            break;
        case ACTIVATE:
            jobQueue.add(this::onActive);
            break;
        case DEACTIVATE:
            jobQueue.add(this::onDeactivate);
            break;
        case ENGAGE:
            jobQueue.add(this::onEngaged);
            break;
        case SHUTDOWN:
            jobQueue.add(this::onShutdown);
            break;
        case PANIC_SHUTDOWN:
            jobQueue.add(this::onPanic);
            break;
        case RESTART:
            jobQueue.add(this::onCleanRestart);
            break;
        default:
            throw new RosRuntimeException(getComponentName() + "received unknow instruction from guidance state machine.");
        }
    }

	@Override
	public ITacticalPlugin getTacticalPluginByName(String pluginName) {
        for (IPlugin p : registeredPlugins) {
            if (p instanceof ITacticalPlugin && p.getVersionInfo().componentName().equals(pluginName)) {
                return (ITacticalPlugin) p;
            }
        }

        return null;
	}
    
}
