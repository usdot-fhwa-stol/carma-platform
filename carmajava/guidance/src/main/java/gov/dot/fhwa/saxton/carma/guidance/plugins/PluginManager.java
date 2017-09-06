/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import org.ros.exception.ServiceException;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import std_msgs.Header;

import java.util.ArrayList;
import java.util.List;

/**
 * Guidance package PluginManager component
 * <p>
 * Responsible for instantiating, running, owning, and runtime management of
 * all plugins installed in the software's operating environment.
 */
public class PluginManager implements Runnable {
    protected final String componentName = "PluginManager";
    protected final long sleepDurationMillis = 30000;
    protected IPubSubService pubSubService;
    protected int sequenceNumber = 0;
    protected ConnectedNode node;

    protected PluginExecutor executor;
    protected PluginServiceLocator pluginServiceLocator;
    protected List<IPlugin> registeredPlugins = new ArrayList<>();

    protected String messagingBaseUrl = "plugins";
    protected String getRegisteredPluginsServiceUrl = "get_registered_plugins";
    protected String getActivePluginsServiceUrl = "get_active_plugin";
    protected String activatePluginServiceUrl = "activate_plugins";

    protected String availablePluginsTopicUrl = "available_plugins";

    protected ServiceServer<PluginListRequest, PluginListResponse> registeredPluginService;
    protected ServiceServer<PluginListRequest, PluginListResponse> activePluginService;
    protected ServiceServer<PluginActivationRequest, PluginActivationResponse>
        activatePluginService;

    public PluginManager(IPubSubService pubSubManager, ConnectedNode node) {
        this.pubSubService = pubSubManager;
        this.node = node;
        this.executor = new PluginExecutor(node.getLog());

        pluginServiceLocator =
            new PluginServiceLocator(new ArbitratorService(), new PluginManagementService(),
                pubSubService, node.getLog());
    }

    /**
     * Load available plugins from the classpath.
     * <p>
     * Presently stubbed to simply load the mock plugins without going through discovery
     */
    protected void discoverPluginsOnClaspath() {
        IPlugin p1 = new MockCruisingPlugin(pluginServiceLocator);
        p1.setActivation(true);
        registeredPlugins.add(p1);
        IPlugin p2 = new MockRouteFollowingPlugin(pluginServiceLocator);
        p2.setActivation(true);
        registeredPlugins.add(p2);
    }

    public List<IPlugin> getRegisteredPlugins() {
        return registeredPlugins;
    }

    /**
     * Configure all the services to respond with dummy data
     */
    private void setupServices() {
        registeredPluginService =
            node.newServiceServer(messagingBaseUrl + "/" + getRegisteredPluginsServiceUrl,
                PluginList._TYPE,
                new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
                    @Override public void build(PluginListRequest pluginListRequest,
                        PluginListResponse pluginListResponse) throws ServiceException {
                        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
                        MessageFactory factory = nodeConfig.getTopicMessageFactory();

                        Header h = factory.newFromType(Header._TYPE);
                        h.setStamp(node.getCurrentTime());
                        h.setFrameId("0");
                        h.setSeq(0);

                        Plugin p0 = factory.newFromType(Plugin._TYPE);
                        p0.setHeader(h);
                        p0.setAvailable(false);
                        p0.setName("DUMMY PLUGIN A");
                        p0.setVersionId("v1.0.0");
                        p0.setActivated(true);

                        Plugin p1 = factory.newFromType(Plugin._TYPE);
                        p1.setHeader(h);
                        p1.setAvailable(true);
                        p1.setName("DUMMY PLUGIN B");
                        p1.setVersionId("v1.0.0");
                        p1.setActivated(false);

                        Plugin p2 = factory.newFromType(Plugin._TYPE);
                        p2.setHeader(h);
                        p2.setAvailable(true);
                        p2.setName("DUMMY PLUGIN A");
                        p2.setVersionId("v2.0.0");
                        p2.setActivated(true);

                        Plugin p3 = factory.newFromType(Plugin._TYPE);
                        p3.setHeader(h);
                        p3.setAvailable(false);
                        p3.setName("DUMMY PLUGIN B");
                        p3.setVersionId("v2.0.0");
                        p3.setActivated(false);

                        List<Plugin> pList = new ArrayList<>();
                        pList.add(p0);
                        pList.add(p1);
                        pList.add(p2);
                        pList.add(p3);

                        pluginListResponse.setPlugins(pList);
                    }
                });

        activePluginService =
            node.newServiceServer(messagingBaseUrl + "/" + getActivePluginsServiceUrl,
                PluginList._TYPE,
                new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
                    @Override public void build(PluginListRequest pluginListRequest,
                        PluginListResponse pluginListResponse) throws ServiceException {
                        NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
                        MessageFactory factory = nodeConfig.getTopicMessageFactory();

                        Header h = factory.newFromType(Header._TYPE);
                        h.setStamp(node.getCurrentTime());
                        h.setFrameId("0");
                        h.setSeq(0);

                        Plugin p0 = factory.newFromType(Plugin._TYPE);
                        p0.setHeader(h);
                        p0.setAvailable(false);
                        p0.setName("DUMMY PLUGIN A");
                        p0.setVersionId("v1.0.0");
                        p0.setActivated(true);

                        Plugin p2 = factory.newFromType(Plugin._TYPE);
                        p2.setHeader(h);
                        p2.setAvailable(true);
                        p2.setName("DUMMY PLUGIN A");
                        p2.setVersionId("v2.0.0");
                        p2.setActivated(true);

                        List<Plugin> pList = new ArrayList<>();
                        pList.add(p0);
                        pList.add(p2);

                        pluginListResponse.setPlugins(pList);
                    }
                });

        activatePluginService =
            node.newServiceServer(messagingBaseUrl + "/" + activatePluginServiceUrl,
                PluginActivation._TYPE,
                new ServiceResponseBuilder<PluginActivationRequest, PluginActivationResponse>() {
                    @Override public void build(PluginActivationRequest pluginActivationRequest,
                        PluginActivationResponse pluginActivationResponse) throws ServiceException {
                        pluginActivationResponse
                            .setNewState(pluginActivationRequest.getActivated());
                    }
                });
    }

    @Override public void run() {
        discoverPluginsOnClaspath();

        for (IPlugin p : getRegisteredPlugins()) {
            executor.submitPlugin(p);
            executor.initializePlugin(p.getName(), p.getVersionId());

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            executor.resumePlugin(p.getName(), p.getVersionId());
        }

        setupServices();
        IPublisher<SystemAlert> pub =
            pubSubService.getPublisherForTopic("system_alert", cav_msgs.SystemAlert._TYPE);

        IPublisher<cav_msgs.PluginList> pluginPublisher = pubSubService
            .getPublisherForTopic(messagingBaseUrl + "/" + availablePluginsTopicUrl,
                cav_msgs.PluginList._TYPE);

        for (; ; ) {
            // Publish system alert status
            cav_msgs.SystemAlert systemAlertMsg = pub.newMessage();
            systemAlertMsg
                .setDescription("Hello World! I am " + componentName + ". " + sequenceNumber++);
            systemAlertMsg.setType(SystemAlert.CAUTION);
            pub.publish(systemAlertMsg);

            NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
            MessageFactory factory = nodeConfig.getTopicMessageFactory();
            cav_msgs.PluginList availablePlugins = factory.newFromType(cav_msgs.PluginList._TYPE);

            // Publish mock available plugin status
            Header h = factory.newFromType(Header._TYPE);
            h.setStamp(node.getCurrentTime());
            h.setFrameId("0");
            h.setSeq(0);

            Plugin p1 = factory.newFromType(Plugin._TYPE);
            p1.setHeader(h);
            p1.setAvailable(true);
            p1.setName("DUMMY PLUGIN B");
            p1.setVersionId("v1.0.0");
            p1.setActivated(true);

            Plugin p2 = factory.newFromType(Plugin._TYPE);
            p2.setHeader(h);
            p2.setAvailable(true);
            p2.setName("DUMMY PLUGIN A");
            p2.setVersionId("v2.0.0");
            p2.setActivated(true);

            List<Plugin> pList = new ArrayList<>();
            pList.add(p1);
            pList.add(p2);

            availablePlugins.setPlugins(pList);
            pluginPublisher.publish(availablePlugins);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
                break; // Fall out of loop if we get interrupted
            }
        }

        // If we're shutting down, properly handle graceful plugin shutdown as well
        for (IPlugin p : getRegisteredPlugins()) {
            executor.suspendPlugin(p.getName(), p.getVersionId());
        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        for (IPlugin p: getRegisteredPlugins()) {
            executor.terminatePlugin(p.getName(), p.getVersionId());
        }
    }
}
