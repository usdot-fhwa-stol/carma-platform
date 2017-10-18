/*
 * TODO: Copyright (C) 2017 LEIDOS
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

package gov.dot.fhwa.saxton.carma.rosutils;

import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.AbstractNodeMain;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.message.Duration;
import org.apache.commons.logging.Log;


/**
 * Extending the ROS Logger functionality for Carma purposes.
 */
public class SaxtonLogger {

  private Log saxtonLog;
  private String source = "NO SOURCE SET";

  /**
   * Get the human readable String representation of the className.
   */
  public void setSource(String sourceName) {
    source = sourceName;
  }
  public String getSource() {
    return source;
  }

  /*
  Initialize the logger.
   */
  public SaxtonLogger(String className, Log connectedNodeLog) {
    this.saxtonLog = connectedNodeLog;
    this.source = className;
  }

  /**
   * The log* methods below were created to leverage the ROS node log and then adds the source and tag from the calling procedure
   * onto the message.
   *
   * @param tag       A string representing the category of the data
   * @param message   A string containing the message to be logged
   */
  public void logInfo(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore);
  }

  public void logInfo(String tag, String message, Throwable t){
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore, t);
  }

  public void logError(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore);
  }

  public void logError(String tag, String message, Throwable t){
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore, t);
  }

  public void logWarn(String tag, String message) {
    String messageToStore =" | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore);
  }

  public void logWarn(String tag, String message, Throwable t){
    String messageToStore =" | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore, t);
  }

  public void logFatal(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore);
  }

  public final void logFatal(String tag, String message, Throwable t){
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore, t);
  }

  public final void logTrace(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore);
  }

  public final void logTrace(String tag, String message, Throwable t){
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore, t);
  }

}