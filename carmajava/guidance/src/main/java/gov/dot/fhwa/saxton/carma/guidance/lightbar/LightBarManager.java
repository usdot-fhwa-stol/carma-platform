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

package gov.dot.fhwa.saxton.carma.guidance.lightbar;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceAction;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceComponent;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceState;
import gov.dot.fhwa.saxton.carma.guidance.GuidanceStateMachine;
import gov.dot.fhwa.saxton.carma.guidance.IStateChangeListener;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnServiceResponseCallback;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.TopicNotFoundException;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.function.BiConsumer;

import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

import cav_msgs.LightBarStatus;
import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import cav_srvs.SetLights;
import cav_srvs.SetLightsRequest;
import cav_srvs.SetLightsResponse;

/**
 * Class Maintains a tracked set of external vehicle paths from MobilityPath and MobilityRequests messages
 * The set of paths can be queried for collisions.
 * 
 * Collision detection is done with a {@link NSpatialHashMap}
 * The path sets are synchronized making this class Thread-Safe
 * 
 * The times stamps used on paths should all be referenced to the same origin
 * The current time information is provided by a passed in {@link IMobilityTimeProvider}
 */
public class LightBarManager   extends GuidanceComponent implements ILightBarManager {
 
  private List<String> controlPriorities;
  private Map<LightBarIndicator, String> lightControlMap = Collections.synchronizedMap(new HashMap<>());
  private Map<String, ILightBarControlChangeHandler> handlerMap = Collections.synchronizedMap(new HashMap<>());
  private IService<SetLightsRequest, SetLightsResponse> lightBarService;
  private LightBarStatus statusMsg;
  private ControlChangeHandler controlChangeHandler = new ControlChangeHandler();
  private final List<LightBarIndicator> ALL_INDICATORS = LightBarIndicator.getListOfAllIndicators();
  private final String LIGHT_BAR_SERVICE = "set_lights";

  /**
   * Constructor
   */
  public LightBarManager(GuidanceStateMachine guidanceStateMachine, IPubSubService pubSubService, ConnectedNode node) {
    super(guidanceStateMachine, pubSubService, node);
    // Load params
    ParameterTree params = node.getParameterTree();
    this.controlPriorities = (List<String>) params.getList("~light_bar_priorities", new LinkedList<String>());
    // Echo params
    log.info("Param light_bar_priorities: " + controlPriorities);
    // Take control of indicators
    takeControlOfIndicators();
  }

  @Override
  public String getComponentName() {
    return "Light Bar Manager";
  }

  @Override
  public void onStartup() {
    // Do Nothing
  }
  @Override
  public void onSystemReady() {
    initLightBarService();
  }
  @Override
  public void onActive() {
    // Do Nothing
  }
  @Override
  public void onEngaged() {
    // Show the center green light as solid
    this.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.SOLID, this.getComponentName());
  }
  @Override
  public void onCleanRestart() {
    turnOffAllLights();
    // Relinquish control of non-center light
    releaseControl(Arrays.asList(LightBarIndicator.YELLOW), this.getComponentName());
  }
  @Override
  public void onDeactivate() {
    turnOffAllLights();
  }

  private class ControlChangeHandler implements ILightBarControlChangeHandler{

    @Override
    public void controlLost(LightBarIndicator lostIndicator) {
      log.info("Lost control of light bar indicator: " + lostIndicator);
    }

  }

  private void takeControlOfIndicators() {
    List<LightBarIndicator> indicators = new ArrayList<>(Arrays.asList(LightBarIndicator.GREEN));
    List<LightBarIndicator> deniedIndicators = this.requestControl(indicators, this.getComponentName(), controlChangeHandler);

    for (LightBarIndicator indicator: deniedIndicators) {
        log.info("Failed to take control of light bar indicator: " + indicator);
    }
  }

  @Override
  public List<LightBarIndicator> requestControl(List<LightBarIndicator> indicators, String requestingComponent, ILightBarControlChangeHandler lightBarChangeHandler) {
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
    if (!controllingComponent.equals(requestingComponent)) {
      log.info(requestingComponent + " failed to set the LightBarIndicator " + indicator + 
      " as this was already controlled by " + controllingComponent);
      return false;
    }

    final String warningString = requestingComponent + " failed to set the LightBarIndicator " + indicator + 
    " as the status" + status + "was unsupported";

    switch(status) {
      case FLASH:
        // TODO:
        break;
      case LEFT_ARROW:
        if (indicator != LightBarIndicator.YELLOW) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setLeftArrow(LightBarStatus.ON);
        break;
      case RIGHT_ARROW:
        if (indicator != LightBarIndicator.YELLOW) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setRightArrow(LightBarStatus.ON); 
      break;
      case SOLID: 
        switch(indicator) {
          case GREEN:
            statusMsg.setGreenSolid(LightBarStatus.ON);
            statusMsg.setGreenFlash(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          case YELLOW:
            log.warn(warningString);
            return false;
          default:
            log.warn(warningString);
            return false;
        }
      break;
      case OFF:
        switch(indicator) {
          case GREEN:
            statusMsg.setGreenSolid(LightBarStatus.OFF);
            statusMsg.setGreenFlash(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          case YELLOW:
            statusMsg.setLeftArrow(LightBarStatus.OFF);
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
        " with status" + status);
      }

      @Override
      public void onFailure(Exception e) {
        log.warn("Failed to set light bar for " + requestingComponent + " for LightBarIndicator " + indicator + 
        " with status" + status + " due to service call error");
      }

    });
    return true;
  }

  private void turnOffAllLights() {
    // Take control of all indicators and turn them off
    this.requestControl(ALL_INDICATORS, this.getComponentName(), controlChangeHandler);
    this.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.OFF, this.getComponentName());
  }

  private void initLightBarService() {
    // Register with the interface manager's service
    IService<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> driverCapabilityService;
    try {
      driverCapabilityService = pubSubService.getServiceForTopic("get_drivers_with_capabilities", GetDriversWithCapabilities._TYPE);
    } catch (TopicNotFoundException tnfe) {
      log.warn("Failed to find GetDriversWithCapabilities service. Not able to control light bar");
      return;
    }

    // Build our request message for longitudinal control drivers
    GetDriversWithCapabilitiesRequest req = (GetDriversWithCapabilitiesRequest) driverCapabilityService.newMessage();

    List<String> reqdCapabilities = new ArrayList<>();
    reqdCapabilities.add(LIGHT_BAR_SERVICE);
    req.setCapabilities(reqdCapabilities);

    // Work around to pass a final object into our anonymous inner class so we can get the
    // response
    final GetDriversWithCapabilitiesResponse[] drivers = new GetDriversWithCapabilitiesResponse[1];
    drivers[0] = null;

    // Call the InterfaceManager to see if we have a driver that matches our requirements
    driverCapabilityService.call(req, new OnServiceResponseCallback<GetDriversWithCapabilitiesResponse>() {
        @Override
        public void onSuccess(GetDriversWithCapabilitiesResponse msg) {
            log.debug("Received GetDriversWithCapabilitiesResponse");
            for (String driverName : msg.getDriverData()) {
                log.debug("Discovered driver: " + driverName);
            }

            drivers[0] = msg;
        }

        @Override
        public void onFailure(Exception e) {
          log.warn("Failed to call GetDriversWithCapabilities service. Not able to control light bar");
        }
    });

    // Verify that the message returned drivers that we can use
    String lightBarServiceName = null;
    if (drivers[0] != null) {
        for (String serviceName : drivers[0].getDriverData()) {
            if (serviceName.endsWith(LIGHT_BAR_SERVICE)) {
              lightBarServiceName = serviceName;
              break;
            }
        }
        if (lightBarServiceName != null) {
          try {
            lightBarService = pubSubService.getServiceForTopic(lightBarServiceName, SetLights._TYPE);
          } catch (TopicNotFoundException e1) {
            log.warn("Failed to find  SetLights service. Not able to control light bar");
          }
        }
    }
  }

  @Override
  public void releaseControl(List<LightBarIndicator> indicators, String requestingComponent) {

    for (LightBarIndicator indicator: indicators) {
      if (indicator == null) {
        log.warn("Tried to release control of null indicator for " + requestingComponent);
        continue;
      }
      // Release control
      String controllingComponent = lightControlMap.get(indicator);
      // If the requester controls this indicator
      if (controllingComponent.equals(requestingComponent)) { 
        // Remove control
        lightControlMap.remove(indicator);
        handlerMap.remove(requestingComponent);
      }
    }
  }
}
