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

import cav_msgs.*;
import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.AbstractNodeMain;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.message.Duration;
import org.ros.node.topic.Publisher;


/**
 * Abstract base class for rosjava nodes used in the carma package.
 */
public abstract class SaxtonBaseNode extends AbstractNodeMain {

  private Publisher<cav_msgs.SystemAlert> systemAlertPublisher;
  private SaxtonLogger log;
  /**
   * Entry point for node once connected to ros network. Wraps the onSaxtonStart method in a try-catch.
   * Hopefully all exceptions thrown by extending nodes will be caught here.
   *
   * @param connectedNode A node instance connected to the ros network
   */
  @Override public final void onStart(ConnectedNode connectedNode) {
    try {
      log = new SaxtonLogger(this.getClass().getSimpleName(), connectedNode.getLog());
      log.info ("STARTUP", "Node called SaxtonBaseNode.onStart().");
      // Define our alert publisher as latching so that recipients are guaranteed to see a message
      // even if it is published before the recipient starts up
      systemAlertPublisher = connectedNode.newPublisher("system_alert", cav_msgs.SystemAlert._TYPE);
      systemAlertPublisher.setLatchMode(true);

      onSaxtonStart(connectedNode);
    } catch (Throwable e) {
      String strace = "\n";
      for (StackTraceElement ste : e.getStackTrace()) {
          strace += "\t" + ste.toString();
          strace += "\n";
      }

      connectedNode.getLog().fatal("Exception reached SaxtonBaseNode onStart function for node " + getDefaultNodeName() + ". ", e);
      handleException(e);
    }
  }

  /**
   * Entry method for all extending nodes. This function should be overridden instead of the onStart method.
   *
   * @param connectedNode A node instance connected to the ros network
   */
  protected abstract void onSaxtonStart(ConnectedNode connectedNode);

  /**
   * Method called when an exception remains uncaught until the SaxtonBaseNode onStart method.
   * Implementing nodes should log the exception and publish the appropriate system alert message
   *
   * @param e The exception to handle
   */
  protected abstract void handleException(Throwable e);

  /**
   * Blocks until the desired service is found and returned or timeout expires. If the timeout expires then returns null.
   * <p>
   * Note: This function should never be called before Definitions of ServiceServers. This will help avoid race conditions.
   *
   * @param service       The name of the ros service
   * @param typeString    The type string defining the service classes. Generally of from std_srvs.SetBool._Type.
   * @param connectedNode The node which is waiting for this service to be available
   * @param timeout       The timeout in milliseconds before this node will cease waiting for this service
   * @param <T>           The service request type such as std_srvs.SetBoolRequest
   * @param <S>           The service response type such as srd_srvs.SetBoolResponse
   * @return An initialized ServiceClient for the desired service
   */
  public final <T, S> ServiceClient<T, S> waitForService(String service, String typeString,
    final ConnectedNode connectedNode, int timeout) {
    ServiceClient<T, S> client = null;
    boolean serviceFound = false;
    Time endTime = connectedNode.getCurrentTime().add(Duration.fromMillis(timeout));
    // Keep searching for service while it is not found and the timeout is not exceeded.
    while (!serviceFound && connectedNode.getCurrentTime().compareTo(endTime) <= 0) {
      try {
        client = connectedNode.newServiceClient(service, typeString);
        serviceFound = true;
      } catch(NullPointerException e) {
    	serviceFound = false;
      } catch (ServiceNotFoundException e) {
        serviceFound = false;
      } catch (RosRuntimeException e) {
        serviceFound = false;
      }
    }
    return client;
  }


  /***
   * Generates a message on the system_alert topic that all nodes listen to.  Typically
   * a handler for a detected broken driver bond.
   * Note that this is not the callback to be provided to the driver's bind service.
   * @param severity - severity of the problem
   * @param message - description of the problem
   * @param e  - throwable exception, leave null if NA
   */

  public void publishSystemAlert(AlertSeverity severity, String message, Throwable e) {
    //in case this method gets called before our onStart is complete, need to check for valid publisher
    if (systemAlertPublisher != null) {

      //convert the severity to the appropriate message type
      SystemAlert alert = systemAlertPublisher.newMessage();
      alert.setType((byte)severity.getVal());

      //set the alert content and send the message
      alert.setDescription(message);

      systemAlertPublisher.publish(alert);

      if (e!=null)
      {
        if (severity == AlertSeverity.FATAL)
          log.fatal("ALERT", "sendSystemAlert: " + severity + ", " + message, e);
        else
          log.info("ALERT", "sendSystemAlert: " + severity + ", " + message, e);
      }
      else{
        if (severity == AlertSeverity.FATAL)
          log.fatal("ALERT", "sendSystemAlert: " + severity + ", " + message);
        else
          log.info("ALERT", "sendSystemAlert: " + severity + ", " + message);
      }
    }
  }

  }