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
public class LightBarStateMachine {
 
  private IService<SetLightsRequest, SetLightsResponse> lightBarService;
  private LightBarStatus statusMsg;
  private ControlChangeHandler controlChangeHandler = new ControlChangeHandler();
  private final List<LightBarIndicator> ALL_INDICATORS = LightBarIndicator.getListOfAllIndicators();
  private final String LIGHT_BAR_SERVICE = "set_lights";
  private final ILogger log;
  private final ILightBarManager lightBarManager;
  private final IPubSubService pubSubService;
  private int stateIdx = 0;
  // State array to assign indices to states
  protected final LightBarState[] states =
  { LightBarState.DISENGAGED, LightBarState.ENGAGED, LightBarState.RECEIVING_MESSAGES, LightBarState.NEGOTIATING};
  // Transition table for state machine
  protected final int[][] transition = {
    /*   STATES: DISENGAGED(0), ENGAGED(1), RECEIVING_MESSAGES(2), NEGOTIATING(3) */
                    /*          EVENTS           */
    { 0, 0, 0, 0 }, /*GUIDANCE_DISENGAGED    */
    { 1, 1, 2, 3 }, /*GUIDANCE_ENGAGED  */
    { 0, 2, 2, 3 }, /*DSRC_MESSAGE_RECEIVED */
    { 0, 3, 3, 3 }, /*NEGOTIATION_UNDERWAY      */
    { 0, 2, 2, 2 }, /*NEGOTIATION_COMPLETE  */
    { 0, 1, 1, 1 }  /*DSRC_MESSAGE_TIMEOUT*/
  };

  /**
   * Constructor
   */
  public LightBarStateMachine(IPubSubService pubSubService, ILightBarManager lightBarManager) {
    // Take control of indicators
    log = LoggerManager.getLogger();
    this.lightBarManager = lightBarManager;
    this.pubSubService = pubSubService;
    takeControlOfIndicators();
  }

  public String getComponentName() {
    return "Light Bar State Machine";
  }

  /**
   * Function which coordinates state transitions and the timeout timers
   *
   * @param event the event which will be used to determine the next state
   */
  public void notify(LightBarEvent event) {
    LightBarState prevState = getState();
    stateIdx = transition[event.ordinal()][stateIdx];
    handleEvent(event, prevState, getState());
    log.info("State = " + getState());
  }

  public void handleEvent(LightBarEvent event, LightBarState prevState, LightBarState newState) {
    switch(event) {
      case GUIDANCE_DISENGAGED:
        if(newState == LightBarState.DISENGAGED) {
          turnOffAllLights();
          // Maybe release control here
        }
        break;
      case GUIDANCE_ENGAGED:
        if (newState == LightBarState.ENGAGED) {
          // Show the center green light as solid
          lightBarManager.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.SOLID, this.getComponentName());
        }
        break;
      case DSRC_MESSAGE_RECEIVED:
        if (newState == LightBarState.RECEIVING_MESSAGES) {
          // Show the center green light as flashing
          lightBarManager.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.FLASH, this.getComponentName());
        }
        break;
      case NEGOTIATION_UNDERWAY:
        if (newState == LightBarState.NEGOTIATING) {
          // Show the yellow lights as flashing
          lightBarManager.setIndicator(LightBarIndicator.YELLOW, IndicatorStatus.FLASH, this.getComponentName());
        }
        break;
      case NEGOTIATION_COMPLETE:
        if (newState == LightBarState.RECEIVING_MESSAGES) {
          // Show the yellows light as off
          lightBarManager.setIndicator(LightBarIndicator.YELLOW, IndicatorStatus.OFF, this.getComponentName());
        }
        break;
      case DSRC_MESSAGE_TIMEOUT:
        if (newState == LightBarState.ENGAGED) {
          // Show the center green light as solid
          lightBarManager.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.SOLID, this.getComponentName());
        }
        break;
    }
  }

  private LightBarState getState() {
    return states[stateIdx];
  }

  private class ControlChangeHandler implements ILightBarControlChangeHandler{

    @Override
    public void controlLost(LightBarIndicator lostIndicator) {
      log.info("Lost control of light bar indicator: " + lostIndicator);
    }

  }

  private void takeControlOfIndicators() {
    List<LightBarIndicator> indicators = new ArrayList<>(Arrays.asList(LightBarIndicator.GREEN));
    List<LightBarIndicator> deniedIndicators = lightBarManager.requestControl(indicators, this.getComponentName(), controlChangeHandler);

    for (LightBarIndicator indicator: deniedIndicators) {
        log.info("Failed to take control of light bar indicator: " + indicator);
    }
  }

  private void turnOffAllLights() {
    // Take control of all indicators and turn them off
    lightBarManager.requestControl(ALL_INDICATORS, this.getComponentName(), controlChangeHandler);
    lightBarManager.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.OFF, this.getComponentName());
  }
}
