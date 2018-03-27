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

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class which serves to coordinate light bar transitions
 * The behavior of this class can be overriden if plugins are given higher lighbar priority
 */
public class LightBarStateMachine implements ILightBarStateMachine {
 
  private ControlChangeHandler controlChangeHandler = new ControlChangeHandler();
  private final List<LightBarIndicator> ALL_INDICATORS = LightBarIndicator.getListOfAllIndicators();
  private final ILogger log;
  private final ILightBarManager lightBarManager;
  private int stateIdx = 0;
  // State array to assign indices to states
  protected final LightBarState[] states =
  { LightBarState.DISENGAGED, LightBarState.ENGAGED, LightBarState.RECEIVING_MESSAGES, LightBarState.NEGOTIATING};
  // Transition table for state machine
  // Columns are states and rows are events
  protected final int[][] transition = {
    /*   STATES: DISENGAGED(0), ENGAGED(1), RECEIVING_MESSAGES(2), NEGOTIATING(3) */
                    /*          EVENTS           */
    { 0, 0, 0, 0 }, /*GUIDANCE_DISENGAGED    */
    { 1, 1, 1, 1 }, /*GUIDANCE_ENGAGED  */
    { 0, 2, 2, 3 }, /*DSRC_MESSAGE_RECEIVED */
    { 0, 3, 3, 3 }, /*NEGOTIATION_UNDERWAY      */
    { 0, 2, 2, 2 }, /*NEGOTIATION_COMPLETE  */
    { 0, 1, 1, 1 }  /*DSRC_MESSAGE_TIMEOUT*/
  };

  /**
   * Constructor
   * 
   * @param lightBarManager The light bar manager which will be called when changes are requested
   */
  public LightBarStateMachine(ILightBarManager lightBarManager) {
    // Take control of indicators
    log = LoggerManager.getLogger();
    this.lightBarManager = lightBarManager;
    takeControlOfIndicators();
  }

  public String getComponentName() {
    return "Light Bar State Machine";
  }

  @Override
  public void next(LightBarEvent event) {
    LightBarState prevState = getCurrentState();
    stateIdx = transition[event.ordinal()][stateIdx];
    handleEvent(event, getCurrentState());
    log.info("Event: " + event + " Prev State: " + prevState + " New State: " + getCurrentState());
  }

  @Override
  public LightBarState getCurrentState() {
    return states[stateIdx];
  }

  /**
   * Helper function for processing incoming events
   * 
   * @param event The event to process
   * @param newState The new state resulting from the event 
   */
  private void handleEvent(LightBarEvent event, LightBarState newState) {
    switch(event) {
      case GUIDANCE_DISENGAGED:
        if(newState == LightBarState.DISENGAGED) {
          turnOffAllLights();
          // Maybe release control here for restart
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

  /**
   * Helper class for processing changes in light control
   */
  private class ControlChangeHandler implements ILightBarControlChangeHandler{

    @Override
    public void controlLost(LightBarIndicator lostIndicator) {
      log.info("Lost control of light bar indicator: " + lostIndicator);
    }

  }

  /**
   * Helper function to take control of the needed indicators
   */
  private void takeControlOfIndicators() {
    List<LightBarIndicator> indicators = new ArrayList<>(Arrays.asList(LightBarIndicator.GREEN));
    List<LightBarIndicator> deniedIndicators = lightBarManager.requestControl(indicators, this.getComponentName(), controlChangeHandler);

    for (LightBarIndicator indicator: deniedIndicators) {
        log.info("Failed to take control of light bar indicator: " + indicator);
    }
  }

  /**
   * Helper function to turnoff all indicators
   */
  private void turnOffAllLights() {
    // Take control of all indicators and turn them off
    lightBarManager.requestControl(ALL_INDICATORS, this.getComponentName(), controlChangeHandler);
    lightBarManager.setIndicator(LightBarIndicator.GREEN, IndicatorStatus.OFF, this.getComponentName());
  }
}
