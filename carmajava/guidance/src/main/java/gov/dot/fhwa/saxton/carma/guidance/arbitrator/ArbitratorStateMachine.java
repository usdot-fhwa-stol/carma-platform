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

package gov.dot.fhwa.saxton.carma.guidance.arbitrator;

import java.util.ArrayList;
import java.util.List;

/**
 * Class responsible for the logic of Arbitrator state transitions
 */
public class ArbitratorStateMachine {
  private ArbitratorState state = ArbitratorState.INIT;
  private List<ArbitratorStateChangeListener> listeners = new ArrayList<>();

  /**
   * Notify the ArbitratorStateMachine that an event has occurred and may cause the
   * state to change.
   */
  public void processEvent(ArbitratorEvent event) {
    ArbitratorState oldState;
    synchronized (this) {
      oldState = state;
      switch (state) {
      case INIT:
        if (event == ArbitratorEvent.INITIALIZE) {
          state = ArbitratorState.INITIAL_PLANNING;
        }
        break;
      case INITIAL_PLANNING:
        if (event == ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING) {
          state = ArbitratorState.AWAITING_REPLAN;
        }
        break;
      case AWAITING_REPLAN:
        if (event == ArbitratorEvent.TRAJECTORY_COMPLETION_ALERT) {
          state = ArbitratorState.NORMAL_REPLANNING;
        } else if (event == ArbitratorEvent.COMPLEX_TRAJECTORY_COMPLETION_ALERT) {
          state = ArbitratorState.REPLAN_AFTER_COMPLEX_TRAJECTORY;
        } else if (event == ArbitratorEvent.TRAJECTORY_FAILED_EXECUTION) {
          state = ArbitratorState.REPLAN_DUE_TO_FAILED_TRAJECTORY;
        }
        break;
      case NORMAL_REPLANNING:
        if (event == ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING) {
          state = ArbitratorState.AWAITING_REPLAN;
        }
        break;
      case REPLAN_AFTER_COMPLEX_TRAJECTORY:
        if (event == ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING) {
          state = ArbitratorState.AWAITING_REPLAN;
        }
        break;
      case REPLAN_DUE_TO_FAILED_TRAJECTORY:
        if (event == ArbitratorEvent.FINISHED_TRAJECTORY_PLANNING) {
          state = ArbitratorState.AWAITING_REPLAN;
        }
        break;
      default:
        throw new IllegalStateException("Arbitrator has detected an unhandled state!!!");
      }
    }
    
    // Call all the listeners if we've changed state
    if (oldState != state) {
      for (ArbitratorStateChangeListener listener : listeners) {
        listener.onStateChange(state);
      }
    }
  }

  /**
   * Get the current state of the arbitrator state machine
   */
  public synchronized ArbitratorState getState() {
    return state;
  }

  /**
   * Register a state change listener to be called when the state changes
   */
  public void registerStateChangeListener(ArbitratorStateChangeListener listener) {
    listeners.add(listener);
  }
}
