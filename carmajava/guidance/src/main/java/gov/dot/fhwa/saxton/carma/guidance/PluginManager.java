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

package gov.dot.fhwa.saxton.carma.guidance;

import cav_msgs.Plugin;
import cav_msgs.SystemAlert;
import cav_srvs.*;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublicationChannel;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.PubSubManager;
import org.ros.exception.ServiceException;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceResponseBuilder;
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
    protected PubSubManager pubSubManager;
    protected int sequenceNumber = 0;
    protected ConnectedNode node;

    protected String serviceRouteUrl = "plugins";
    protected String getRegisteredPluginsServiceUrl = "getRegisteredPlugins";
    protected String getAvailablePluginsServiceUrl = "getAvailablePlugins";
    protected String getActivePluginsServiceUrl = "getActivePlugins";
    protected String activatePluginServiceUrl = "activatePlugin";

    public PluginManager(PubSubManager pubSubManager, ConnectedNode node) {
        this.pubSubManager = pubSubManager;
        this.node = node;
    }

    /**
     * Configure all the services to respond with dummy data
     */
    private void setupServices() {
        node.newServiceServer(serviceRouteUrl + "/" + getRegisteredPluginsServiceUrl,
            PluginList._TYPE, new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
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

        node.newServiceServer(serviceRouteUrl + "/" + getActivePluginsServiceUrl, PluginList._TYPE,
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

        node.newServiceServer(serviceRouteUrl + "/" + getAvailablePluginsServiceUrl,
            PluginList._TYPE, new ServiceResponseBuilder<PluginListRequest, PluginListResponse>() {
                @Override public void build(PluginListRequest pluginListRequest,
                    PluginListResponse pluginListResponse) throws ServiceException {
                    NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
                    MessageFactory factory = nodeConfig.getTopicMessageFactory();

                    Header h = factory.newFromType(Header._TYPE);
                    h.setStamp(node.getCurrentTime());
                    h.setFrameId("0");
                    h.setSeq(0);

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

                    List<Plugin> pList = new ArrayList<>();
                    pList.add(p1);
                    pList.add(p2);

                    pluginListResponse.setPlugins(pList);
                }
            });

        node.newServiceServer(serviceRouteUrl + "/" + activatePluginServiceUrl,
            PluginActivation._TYPE,
            new ServiceResponseBuilder<PluginActivationRequest, PluginActivationResponse>() {
                @Override public void build(PluginActivationRequest pluginActivationRequest,
                    PluginActivationResponse pluginActivationResponse) throws ServiceException {
                    pluginActivationResponse.setNewState(pluginActivationRequest.getActivated());
                }
            });
    }

    @Override public void run() {
        setupServices();

        IPublicationChannel<SystemAlert> pub =
            pubSubManager.getPublicationChannelForTopic("system_alert", cav_msgs.SystemAlert._TYPE);

        for (;;) {
            cav_msgs.SystemAlert systemAlertMsg = pub.newMessage();
            systemAlertMsg
                .setDescription("Hello World! I am " + componentName + ". " + sequenceNumber++);
            systemAlertMsg.setType(SystemAlert.CAUTION);
            pub.publish(systemAlertMsg);

            try {
                Thread.sleep(sleepDurationMillis);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
    }
}
