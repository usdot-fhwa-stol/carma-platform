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

package gov.dot.fhwa.saxton.carma.guidance;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;

import java.util.concurrent.atomic.AtomicReference;
import org.ros.node.ConnectedNode;

import cav_msgs.RouteState;
import cav_msgs.SystemAlert;

/**
 * Handles final cleanup after Guidance shutdown
 * <p>
 * Also listens for route_state changes that would indicate the need for a Guidance shutdown
 */
public class GuidanceShutdownHandler extends GuidanceComponent {
  protected long shutdownDelayMs = 5000;
  protected ISubscriber<RouteState> routeStateSub;
  protected IPublisher<SystemAlert> systemAlertPub;

  public GuidanceShutdownHandler(AtomicReference<GuidanceState> state, IPubSubService pubSubService, ConnectedNode node) {
    super(state, pubSubService, node);
  }

  @Override
  public String getComponentName() {
    return "GuidanceShutdownHandler";
  }

  @Override
  public void onGuidanceStartup() {
    shutdownDelayMs = node.getParameterTree().getInteger("shutdown_wait_time", 5000);

    systemAlertPub = pubSubService.getPublisherForTopic("system_alert", SystemAlert._TYPE);

    routeStateSub = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSub.registerOnMessageCallback(new OnMessageCallback<RouteState>(){
      @Override
      public void onMessage(RouteState msg) {
        if (msg.getState() == RouteState.LEFT_ROUTE) {
          log.info("Recieved route state LEFT_ROUTE, beginning FATAL shutdown procedures");
          SystemAlert alert = systemAlertPub.newMessage();
          alert.setDescription("Guidance detected LEFT_ROUTE state!");
          alert.setType(SystemAlert.FATAL);
          systemAlertPub.publish(alert);
        } else if (msg.getState() == RouteState.ROUTE_COMPLETED || msg.getState() == RouteState.ROUTE_ABORTED) {
          log.info("Recieved route state ROUTE_COMPLETED or ROUTE_ABORTED, beginning normal shutdown procedures");
          SystemAlert alert = systemAlertPub.newMessage();
          alert.setDescription("Guidance detected ROUTE_COMPLETED or ROUTE_ABORTED state.");
          alert.setType(SystemAlert.SHUTDOWN);
          systemAlertPub.publish(alert);
        }
      }
    });
  }

  @Override
  public void onSystemReady() {
    // NO-OP
  }

  @Override
  public void onGuidanceEnable() {
    // NO-OP
  }

  @Override
  public void onGuidanceShutdown() {
    log.info("SHUTDOWN", "Guidance shutdown handler waiting" + shutdownDelayMs + "ms for guidance thread shutdown...");

    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      log.error("SHUTDOWN", "Guidance shutdown handler interrupted while waiting, cleanup may not have finished!", e);
    }

    log.info("SHUTDOWN", "Guidance shutdown handler killing Guidance node.");
    node.shutdown();
  }
}