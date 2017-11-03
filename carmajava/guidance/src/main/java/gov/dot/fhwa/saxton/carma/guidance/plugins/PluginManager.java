/*
 * Copyright (C) 2017 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.params.RosParameterSource;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.ManeuverPlanner;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import org.reflections.Reflections;
import org.ros.exception.ServiceException;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import std_msgs.Header;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Guidance package PluginManager component
 * <p>
 * Responsible for instantiating, running, owning, and runtime management of
 * all plugins installed in the software's operating environment.
 */
public class PluginManager extends GuidanceComponent implements AvailabilityListener {
    protected final long sleepDurationMillis = 30000;
    protected int sequenceNumber = 0;

    protected PluginExecutor executor;
    protected PluginServiceLocator pluginServiceLocator;
    protected List<IPlugin> registeredPlugins = new ArrayList<>();
    protected List<String> ignoredPluginClassNames = new ArrayList<>();
    protected List<String> requiredPluginClassNames = new ArrayList<>();

    protected final String PLUGIN_DISCOVERY_ROOT = "gov.dot.fhwa.saxton.carma.guidance.plugins";
    protected final String messagingBaseUrl = "plugins";
    protected final String getRegisteredPluginsServiceUrl = "get_registered_plugins";
    protected final String getActivePluginsServiceUrl = "get_active_plugins";
    protected final String activatePluginServiceUrl = "activate_plugin";
    protected final String availablePluginsTopicUrl = "available_plugins";

    protected ServiceServer<PluginListRequest, PluginListResponse> registeredPluginService;
    protected ServiceServer<PluginListRequest, PluginListResponse> activePluginService;
    protected ServiceServer<PluginActivationRequest, PluginActivationResponse> activatePluginService;
    protected IPublisher<cav_msgs.PluginList> pluginPublisher;

    protected MessageFactory messageFactory;

    protected int availablePluginsSeqNum = 0;
    protected int registeredPluginsSeqNum = 0;
    protected int activePluginsSeqNum = 0;

    public PluginManager(AtomicReference<GuidanceState> state, IPubSubService pubSubManager, 
    IGuidanceCommands commands, IManeuverInputs maneuverInputs, ConnectedNode node) {
        super(state, pubSubManager, node);
        this.executor = new PluginExecutor();

        pluginServiceLocator = new PluginServiceLocator(new ArbitratorService(), new PluginManagementService(),
                pubSubService, new RosParameterSource(node.getParameterTree()), new ManeuverPlanner(commands, maneuverInputs));
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
                log.info("PLUGIN", "Guidance.PluginManager instantiated new plugin instance: " + pluginInstance.getName() + ":"
                        + pluginInstance.getVersionId());

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
    public void onGuidanceStartup() {
        // Instantiate the plugins and register them
        ignoredPluginClassNames = (List<String>) node.getParameterTree().getList("~ignored_plugins", new ArrayList<>());
        requiredPluginClassNames = (List<String>) node.getParameterTree().getList("~required_plugins",
                new ArrayList<>());

        log.info("STARTUP", "Ignoring plugins: " + ignoredPluginClassNames.toString());
        log.info("STARTUP", "Requiring plugins: " + requiredPluginClassNames.toString());
        List<Class<? extends IPlugin>> pluginClasses = discoverPluginsOnClasspath();

        registeredPlugins = instantiatePluginsFromClasses(pluginClasses, pluginServiceLocator);
        for (IPlugin p : getRegisteredPlugins()) {
            executor.submitPlugin(p);
            executor.initializePlugin(p.getName(), p.getVersionId());

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Setup the services related to plugin queries
        setupServices();

        // Configure the plugin availability topic and topic message factory
        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
        messageFactory = nodeConfig.getTopicMessageFactory();
        pluginPublisher = pubSubService.getPublisherForTopic(messagingBaseUrl + "/" + availablePluginsTopicUrl,
                cav_msgs.PluginList._TYPE);
    }

    @Override
    public void onSystemReady() {
    }

    @Override
    public void onGuidanceEnable() {
        for (IPlugin p : getRegisteredPlugins()) {
            executor.resumePlugin(p.getName(), p.getVersionId());
        }
    }

    @Override
    public void onGuidanceShutdown() {
        // If we're shutting down, properly handle graceful plugin shutdown as well
        for (IPlugin p : getRegisteredPlugins()) {
            p.setActivation(false);
            executor.suspendPlugin(p.getName(), p.getVersionId());
        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        for (IPlugin p : getRegisteredPlugins()) {
            executor.terminatePlugin(p.getName(), p.getVersionId());
        }
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
                pMsg.setAvailable(p.getAvailability());
                pMsg.setName(p.getName());
                pMsg.setVersionId(p.getVersionId());
                pMsg.setActivated(p.getActivation());
                pMsg.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                pList.add(pMsg);
            }
        }

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
                            p0.setAvailable(p.getAvailability());
                            p0.setName(p.getName());
                            p0.setVersionId(p.getVersionId());
                            p0.setActivated(p.getActivation());
                            p0.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                            pList.add(p0);
                        }

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
                                p0.setAvailable(p.getAvailability());
                                p0.setName(p.getName());
                                p0.setVersionId(p.getVersionId());
                                p0.setActivated(p.getActivation());
                                p0.setRequired(requiredPluginClassNames.contains(p.getClass().getName()));
                                pList.add(p0);
                            }
                        }

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
                        boolean pluginFound = false;
                        for (IPlugin p : registeredPlugins) {
                            if (pluginActivationRequest.getPluginName().equals(p.getName())
                                    && pluginActivationRequest.getPluginVersion().equals(p.getVersionId())) {
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
}
