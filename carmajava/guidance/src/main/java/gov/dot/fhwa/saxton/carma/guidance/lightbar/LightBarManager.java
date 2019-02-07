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

package gov.dot.fhwa.saxton.carma.guidance.lightbar;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceAction;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.IStateChangeListener;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

import cav_msgs.BSM;
import cav_msgs.LightBarStatus;
import cav_srvs.SetLights;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;

/**
 * Guidance component which exposes control of the light bar to plugins
 * Maintains its own light bar state machine which can be overruled by plugins using a parameter
 */
public class LightBarManager extends GuidanceComponent implements IStateChangeListener, ILightBarManager, ILightBarStateMachine {
 
  private List<String> controlPriorities;
  private Map<LightBarIndicator, String> lightControlMap = Collections.synchronizedMap(new HashMap<>());
  private Map<String, ILightBarControlChangeHandler> handlerMap = Collections.synchronizedMap(new HashMap<>());
  private IService<SetLightsRequest, SetLightsResponse> lightBarService;
  private LightBarStatus statusMsg;
  private final String BSM_TOPIC = "bsm";
  private final LightBarStateMachine lightBarStateMachine;
  private final ISubscriber<BSM> bsmTopic;
  private long lastBSM = 0;
  private long TIMEOUT_MS = 1000;
  private AtomicBoolean haveRecentBSM = new AtomicBoolean();

  /**
   * Constructor
   * 
   * @param guidanceStateMachine The guidance state machine
   * @param pubSubService An instance of an IPubSubService to extract the set_lights service from
   * @param node A rose node which contains this object
   */
  public LightBarManager(GuidanceStateMachine guidanceStateMachine, IPubSubService pubSubService, ConnectedNode node) {
    super(guidanceStateMachine, pubSubService, node);
    // Load params
    ParameterTree params = node.getParameterTree();
    this.controlPriorities = (List<String>) params.getList("~light_bar_priorities", new LinkedList<String>());
    this.TIMEOUT_MS = params.getInteger("~light_bar_comms_timeout", (int) TIMEOUT_MS);
    // Echo params
    log.info("Param light_bar_priorities: " + controlPriorities);
    log.info("Param light_bar_comms_timeout: " + TIMEOUT_MS);
    // Init State Machine
    lightBarStateMachine = new LightBarStateMachine(this);
    // Get incoming bsm topic
    lastBSM = System.currentTimeMillis();
    bsmTopic = pubSubService.getSubscriberForTopic(BSM_TOPIC, BSM._TYPE);
    bsmTopic.registerOnMessageCallback(
      (BSM message) -> {
        if (!haveRecentBSM.get()) { // Only notify state machine of change when this is the first message after a timeout
          lightBarStateMachine.next(LightBarEvent.DSRC_MESSAGE_RECEIVED);
          haveRecentBSM.set(true);
        }
        lastBSM = System.currentTimeMillis();
    });

    guidanceStateMachine.registerStateChangeListener(this);
  }

  //// ILightBarStateMachine Methods
  @Override
  public void next(LightBarEvent event) {
    lightBarStateMachine.next(event); 
  }

  @Override
  public LightBarState getCurrentState() {
    return lightBarStateMachine.getCurrentState();
  }

  //// ILightBarManager Methods
  @Override
  public String getComponentName() {
    return "Light Bar Manager";
  }

  @Override
  public void onStartup() {
  }

  @Override
  public void onSystemReady() {
    initLightBarService(); // Get the light bar service
  }

  @Override
  public void onActive() {
  }

  @Override
  public void onEngaged() {
    lightBarStateMachine.next(LightBarEvent.GUIDANCE_ENGAGED);
  }
  
  @Override
  public void onCleanRestart() {
    lightBarStateMachine.next(LightBarEvent.GUIDANCE_DISENGAGED);
  }

  @Override
  public void onDeactivate() {
    lightBarStateMachine.next(LightBarEvent.GUIDANCE_DISENGAGED);
  }

  @Override
  public void timingLoop() throws InterruptedException {
    try {
        Thread.sleep(TIMEOUT_MS);
        if (System.currentTimeMillis() - lastBSM > TIMEOUT_MS && lightBarService != null) {
          lightBarStateMachine.next(LightBarEvent.DSRC_MESSAGE_TIMEOUT);
          haveRecentBSM.set(false);
        }
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw e;
    }
}

  @Override
  public synchronized List<LightBarIndicator> requestControl(List<LightBarIndicator> indicators, String requestingComponent, ILightBarControlChangeHandler lightBarChangeHandler) {
    List<LightBarIndicator> deniedIndicators = new LinkedList<>();
    // Attempt to acquire control of all indicators
    for (LightBarIndicator indicator: indicators) {
      if (indicator == null) {
        log.warn("Control of a null light bar indicator was requested by " + requestingComponent);
        continue;
      }
      // Attempt control
      String controllingComponent = lightControlMap.get(indicator);
      if (controllingComponent == null) { // If no other component has claimed this indicator
        // Add new controller
        lightControlMap.put(indicator, requestingComponent);
        handlerMap.put(requestingComponent, lightBarChangeHandler);
      } else if (!controllingComponent.equals(requestingComponent)) { // If this indicator is already controlled
        // If the requesting component has higher priority it may take control of this indicator
        if (hasHigherPriority(requestingComponent, controllingComponent)) {
          // Add new controller
          lightControlMap.put(indicator, requestingComponent);
          handlerMap.put(requestingComponent, lightBarChangeHandler);
          //call handler of previous controller
          handlerMap.get(controllingComponent).controlLost(indicator);
        } else {
          deniedIndicators.add(indicator); // Notify caller of failure to take control of component
        }
      }
    }
    return deniedIndicators;
  }

  /**
   * Helper function for comparing two light bar controllers
   * 
   * @param requester The component requesting control from the controller
   * @param controller The component currently in control of the relevant indicator
   * 
   * @return True if the requester has higher priority than the controller
   */
  private boolean hasHigherPriority(String requester, String controller) {
    final int requesterPriority = controlPriorities.indexOf(requester);
    final int controllerPriority = controlPriorities.indexOf(controller);

    // Components not in the priority list are assumed to have lowest priority
    if (requesterPriority < 0) {
      log.warn(requester + " tried to set the light bar but is not in the priority list");
      return false;
    } else if (controllerPriority < 0) {
      log.warn(controller + " a component controls the light bar but is not in the priority list");
      return true; 
    }
    return requesterPriority < controllerPriority;
  }

  @Override
  public boolean setIndicator(LightBarIndicator indicator, IndicatorStatus status, String requestingComponent) {
    String controllingComponent = lightControlMap.get(indicator);
    // Check if the requester has control of this light
    if (controllingComponent != null && !controllingComponent.equals(requestingComponent)) {
      log.info(requestingComponent + " failed to set the LightBarIndicator " + indicator + 
      " as this was already controlled by " + controllingComponent);
      return false;
    }

    if (lightBarService == null) {
      log.info(requestingComponent + " failed to set the LightBarIndicator " + indicator + 
      " as lightBarService was null");
      return false;
    }
    
    if (statusMsg == null) {
      statusMsg = lightBarService.newMessage().getSetState();
    }

    final String warningString = requestingComponent + " failed to set the LightBarIndicator " + indicator + 
    " as the status" + status + "was unsupported";
    // TODO brake this big switch statement out if possible
    // Set indicator and validate request
    // TODO brake this big switch statement out if possible
    switch(status) {
      case FLASH:
        switch(indicator) {
          case GREEN:
            statusMsg.setGreenFlash(LightBarStatus.ON);
            statusMsg.setGreenSolid(LightBarStatus.OFF);
            break;
          case YELLOW:
            statusMsg.setFlash(LightBarStatus.ON);
            statusMsg.setLeftArrow(LightBarStatus.OFF);
            statusMsg.setRightArrow(LightBarStatus.OFF);
            break;
          default:
            log.warn(warningString);
            return false;
        }
        break;
      case LEFT_ARROW:
        if (indicator != LightBarIndicator.YELLOW) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setFlash(LightBarStatus.OFF);
        statusMsg.setRightArrow(LightBarStatus.OFF);
        statusMsg.setLeftArrow(LightBarStatus.ON);
        break;
      case RIGHT_ARROW:
        if (indicator != LightBarIndicator.YELLOW) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setFlash(LightBarStatus.OFF);
        statusMsg.setRightArrow(LightBarStatus.ON); 
        statusMsg.setLeftArrow(LightBarStatus.OFF);
        break;
      case SOLID: 
        if (indicator != LightBarIndicator.GREEN) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setGreenSolid(LightBarStatus.ON);
        statusMsg.setGreenFlash(LightBarStatus.OFF); 
        break;
      case OFF:
        switch(indicator) {
          case GREEN:
            statusMsg.setGreenSolid(LightBarStatus.OFF);
            statusMsg.setGreenFlash(LightBarStatus.OFF);
            break;
          case YELLOW:
            statusMsg.setLeftArrow(LightBarStatus.OFF);
            statusMsg.setRightArrow(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          default:
            log.warn(warningString);
            return false;
        }
        break;
      default:
        log.warn(warningString);
        return false;
    }
    // Take down state is currently unsupported
    statusMsg.setTakedown(LightBarStatus.OFF);
    // Publish new status
    SetLightsRequest req = lightBarService.newMessage();
    req.setSetState(statusMsg);
    lightBarService.call(req, new OnServiceResponseCallback<SetLightsResponse>() {

      @Override
      public void onSuccess(SetLightsResponse msg) {
        log.info("Set light bar for " + requestingComponent + " for LightBarIndicator " + indicator + 
        " with status " + status);
      }

      @Override
      public void onFailure(Exception e) {
        log.warn("Failed to set light bar for " + requestingComponent + " for LightBarIndicator " + indicator + 
        " with status" + status + " due to service call error");
      }

    });
    return true;
  }

  /**
   * Helper function for initializing the set_lights service
   * Uses the interface manager to find the required service
   */
  private void initLightBarService() {
    try {
  	  lightBarService = pubSubService.getServiceForTopic("set_lights", SetLights._TYPE);
    } catch (TopicNotFoundException e1) {
      log.warn("Failed to find  SetLights service. Not able to control light bar");
    }
  }

  @Override
  public synchronized void releaseControl(List<LightBarIndicator> indicators, String requestingComponent) {

    for (LightBarIndicator indicator: indicators) {
      if (indicator == null) {
        log.warn("Tried to release control of null indicator for " + requestingComponent);
        continue;
      }
      // Release control
      String controllingComponent = lightControlMap.get(indicator);
      // If the requester controls this indicator
      if (controllingComponent != null && controllingComponent.equals(requestingComponent)) { 
        // Remove control
        lightControlMap.remove(indicator);
        // Check if the requesting component still controls any indicators
        // Remove the handler if it does
        if (!lightControlMap.containsValue(requestingComponent)) {
          handlerMap.remove(requestingComponent);
        }
      }
    }
  }

  /*
    * This method add the right job in the jobQueue base on the instruction given by GuidanceStateMachine
    * The actual changing of GuidanceState local copy is happened when each job is performed
    */
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
        log.warn(getComponentName() + "received unknown instruction from guidance state machine.");
    }
  }
}
