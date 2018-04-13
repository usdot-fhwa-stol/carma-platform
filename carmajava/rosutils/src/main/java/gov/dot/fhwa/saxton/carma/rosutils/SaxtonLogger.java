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
  private File file = null;
  private String fileName;
  private final String debugLevel = "DEBUG";
  private final String infoLevel = "INFO";
  private final String warnLevel = "WARN";
  private final String errorLevel = "ERROR";
  private final String fatalLevel = "FATAL";
  private final String traceLevel = "TRACE";


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

    try {
      //Initial setup requires performing 2 commands on the terminal.
      //1) sudo mkdir -p /opt/carma/logs and
      //2) sudo chmod -R ugo+rw /opt/carma
      //log file name is generated and stored in /opt/carma/logname.txt
      BufferedReader br = new BufferedReader(new FileReader("/opt/carma/logname.txt")); 
      fileName = br.readLine() + ".txt";
      file = new File("/opt/carma/logs/" + fileName); //TODO: Will see later if needed to be stored in param.
      file.getParentFile().mkdirs();
      br.close();
    } catch (Exception e) {

      //Ignore but do log it.
      saxtonLog.info("SaxtonLogger main function caught an exception: ", e);
    }
  }


  /**
   * The log methods below were created to leverage the ROS node log and then adds the source and tag from the calling procedure
   * onto the message.
   */
  public void debug(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.debug(messageToStore);
    writeToFile(" | " + debugLevel + messageToStore);

  }

  public void debug(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.debug(messageToStore, t);
    writeToFile(" | " + debugLevel + messageToStore, t);
  }

  public void debug(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.debug(messageToStore);
    writeToFile(" | " + debugLevel + messageToStore);

  }

  public void debug(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.debug(messageToStore, t);
    writeToFile(" | " + debugLevel + messageToStore, t);
  }

  public void info(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.info(messageToStore);
    writeToFile(" | " + infoLevel + messageToStore);

  }

  public void info(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.info(messageToStore, t);
    writeToFile(" | " + infoLevel + messageToStore, t);
  }

  public void info(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore);
    writeToFile(" | " + infoLevel + messageToStore);

  }

  public void info(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.info(messageToStore, t);
    writeToFile(" | " + infoLevel + messageToStore, t);
  }

  public void warn(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.warn(messageToStore);
    writeToFile(" | " + warnLevel + messageToStore);
  }

  public void warn(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.warn(messageToStore, t);
    writeToFile(" | " + warnLevel + messageToStore, t);
  }

  public void warn(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore);
    writeToFile(" | " + warnLevel + messageToStore);
  }

  public void warn(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.warn(messageToStore, t);
    writeToFile(" | " + warnLevel + messageToStore, t);
  }

  public void error(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.error(messageToStore);
    writeToFile(" | " + errorLevel + messageToStore);
  }

  public void error(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.error(messageToStore, t);
    writeToFile(" | " + errorLevel + messageToStore, t);
  }

  public void error(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore);
    writeToFile(" | " + errorLevel + messageToStore);
  }

  public void error(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.error(messageToStore, t);
    writeToFile(" | " + errorLevel + messageToStore, t);
  }

  public void fatal(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.fatal(messageToStore);
    writeToFile(" | " + fatalLevel + messageToStore);
  }

  public void fatal(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.fatal(messageToStore, t);
    writeToFile(" | " + fatalLevel + messageToStore, t);
  }

  public void fatal(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore);
    writeToFile(" | " + fatalLevel + messageToStore);
  }

  public void fatal(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.fatal(messageToStore, t);
    writeToFile(" | " + fatalLevel + messageToStore, t);
  }

  public void trace(String message) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.trace(messageToStore);
    writeToFile(" | " + traceLevel + messageToStore);
  }

  public void trace(String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + emptyTag + " | " + message;
    saxtonLog.trace(messageToStore, t);
    writeToFile(" | " + traceLevel + messageToStore, t);
  }

  public void trace(String tag, String message) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore);
    writeToFile(" | " + traceLevel + messageToStore);
  }

  public void trace(String tag, String message, Throwable t) {
    String messageToStore = " | " + getSource() + " | " + tag + " | " + message;
    saxtonLog.trace(messageToStore, t);
    writeToFile(" | " + traceLevel + messageToStore, t);
  }


  /***
   * Write the log to a file with no exceptions.
   * @param message
   */
  private void writeToFile(String message) {
      writeToFile(message, null);
  }

  /***
   * Write the logs to a file with exception.
   * @param message
   * @param exception : Optional, set to null.
   */
  private void writeToFile(String message, Throwable exception) {

    //try-with-resources
    try (FileWriter fw = new FileWriter(file.getAbsoluteFile(), true);
         BufferedWriter bw = new BufferedWriter(fw);
         PrintWriter pw = new PrintWriter(bw);) {

      //Using local date time for now, since this is just for logging purpose. Will see if need to pass in the entire connectedNode to get the time.
      DateTimeFormatter dateTimeformatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss:SSS");
      String formatDateTime = LocalDateTime.now().format(dateTimeformatter);

      bw.append(formatDateTime + message);
      bw.newLine();

      if (exception !=null )
        exception.printStackTrace(pw);

    } catch (IOException e) {

      //Ignore but do log it.
      saxtonLog.info("SaxtonLogger printToLogFile failed (catch): ", e);

    }
  }// end of writeToFile()

  /**
   * Returns the Log instance used for logging in this instance of SaxtonLogger
   * This can be used to pass the logger between classes without overriding the class names
   */
  public Log getBaseLoggerObject() {
    return saxtonLog;
  }
}