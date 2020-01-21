/*
 * Copyright (C) 2018-2020 LEIDOS.
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

import org.apache.commons.logging.Log;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.PrintWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;


/**
 * Extending the ROS Logger functionality for Carma purposes.
 */
public class SaxtonLogger {

  private Log saxtonLog;
  private String source = "NONE";
  private String emptyTag = "NONE";

  /***
   * Get source name which is usually the className.
   * @param sourceName
   */
  public void setSource(String sourceName) {
    source = sourceName;
  }

  /***
   * Get the source name.
   * @return
   */
  public String getSource() {
    return source;
  }

  /***
   * Initialize the logger.
   * @param className
   * @param connectedNodeLog
   */
  public SaxtonLogger(String className, Log connectedNodeLog) {
    this.saxtonLog = connectedNodeLog;
    this.source = className;
  }


  /**
   * The log methods below were created to leverage the ROS node log and then adds the source and tag from the calling procedure
   * onto the message.
   */
  public void debug(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.debug(messageToStore);
  }

  public void debug(String message, Throwable t) {
    String messageToStore =  getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.debug(messageToStore, t);
  }

  public void debug(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.debug(messageToStore);
  }

  public void debug(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.debug(messageToStore, t);
  }

  public void info(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.info(messageToStore);
  }

  public void info(String message, Throwable t) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.info(messageToStore, t);
  }

  public void info(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore);
  }

  public void info(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore, t);
  }

  public void warn(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.warn(messageToStore);
  }

  public void warn(String message, Throwable t) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.warn(messageToStore, t);
  }

  public void warn(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore);
  }

  public void warn(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore, t);
  }

  public void error(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.error(messageToStore);
  }

  public void error(String message, Throwable t) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.error(messageToStore, t);
  }

  public void error(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore);
  }

  public void error(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore, t);
  }

  public void fatal(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.fatal(messageToStore);
  }

  public void fatal(String message, Throwable t) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.fatal(messageToStore, t);
  }

  public void fatal(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore);
  }

  public void fatal(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore, t);
  }

  public void trace(String message) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.trace(messageToStore);
  }

  public void trace(String message, Throwable t) {
    String messageToStore = getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.trace(messageToStore, t);
  }

  public void trace(String tag, String message) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore);
  }

  public void trace(String tag, String message, Throwable t) {
    String messageToStore = getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore, t);
  }

  /**
   * Returns the Log instance used for logging in this instance of SaxtonLogger
   * This can be used to pass the logger between classes without overriding the class names
   */
  public Log getBaseLoggerObject() {
    return saxtonLog;
  }
}