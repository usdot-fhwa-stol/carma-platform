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

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
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

import cav_msgs.LightBarStatus;

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
public class LightBarManager implements ILightBarManager {
 
  private List<String> controlPriorities = new ArrayList<>();
  private Map<LightBarIndicator, String> lightControlMap = Collections.synchronizedMap(new HashMap<>());
  private Map<String, ILightBarControlChangeHandler> handlerMap = Collections.synchronizedMap(new HashMap<>());
  private final IPublisher<LightBarStatus> lightBarPub;
  private final ILogger log;
  private LightBarStatus statusMsg;

  /**
   * Constructor
   */
  public LightBarManager(IPublisher<LightBarStatus> lightBarPub) {
    this.lightBarPub = lightBarPub;
    this.log = LoggerManager.getLogger();
    this.statusMsg = lightBarPub.newMessage();
  }



  @Override
  public List<LightBarIndicator> requestControl(List<LightBarIndicator> indicators, ILightBarControlChangeHandler lightBarChangeHandler, String requestingComponent) {
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
        if (indicator != LightBarIndicator.LEFT_INDICATOR) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setLeftArrow(LightBarStatus.ON);
        break;
      case RIGHT_ARROW:
        if (indicator != LightBarIndicator.RIGHT_INDICATOR) {
          log.warn(warningString);
          return false;
        }
        statusMsg.setRightArrow(LightBarStatus.ON); 
      break;
      case SOLID: 
        switch(indicator) {
          case CENTER_INDICATOR:
            statusMsg.setGreenSolid(LightBarStatus.ON);
            statusMsg.setGreenFlash(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          case LEFT_INDICATOR:
            log.warn(warningString);
            return false;
          case RIGHT_INDICATOR:
            log.warn(warningString);
            return false;
          default:
            log.warn(warningString);
            return false;
        }
      break;
      case OFF:
        switch(indicator) {
          case CENTER_INDICATOR:
            statusMsg.setGreenSolid(LightBarStatus.OFF);
            statusMsg.setGreenFlash(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          case LEFT_INDICATOR:
            statusMsg.setLeftArrow(LightBarStatus.OFF);
            statusMsg.setFlash(LightBarStatus.OFF);
            break;
          case RIGHT_INDICATOR:
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
    lightBarPub.publish(statusMsg);
    return true;
  }

}
