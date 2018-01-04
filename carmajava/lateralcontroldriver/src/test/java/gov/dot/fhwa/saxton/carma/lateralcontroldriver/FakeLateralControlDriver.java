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

package gov.dot.fhwa.saxton.carma.lateralcontroldriver;

import org.ros.message.Time;

public class FakeLateralControlDriver implements ILateralControlDriver{

  private boolean leftTurnMsgReceived = false;
  private boolean rightTurnMsgReceived = false;

  @Override public void publishUIMessage(cav_msgs.UIInstructions msg) {
    if (msg.getMsg().contains("LEFT_TURN")) {
      leftTurnMsgReceived = true;
    } else if (msg.getMsg().contains("RIGHT_TURN")) {
      rightTurnMsgReceived = true;
    }
  }

  public boolean getRightTurnMsgReceived() {
    return rightTurnMsgReceived;
  }

  public boolean getLeftTurnMsgReceived() {
    return leftTurnMsgReceived;
  }

  public void resetFlags() {
    leftTurnMsgReceived = false;
    rightTurnMsgReceived = false;
  }

  @Override public Time getTime() {
    return new Time();
  }
}
