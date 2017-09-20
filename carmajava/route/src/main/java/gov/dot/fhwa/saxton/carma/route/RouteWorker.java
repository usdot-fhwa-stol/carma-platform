/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
package gov.dot.fhwa.saxton.carma.route;

import cav_msgs.RouteState;
import cav_msgs.SystemAlert;
import cav_srvs.GetAvailableRoutesResponse;
import cav_srvs.SetActiveRouteRequest;
import cav_srvs.SetActiveRouteResponse;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import sensor_msgs.NavSatFix;
import sensor_msgs.NavSatStatus;

import java.io.File;
import java.util.*;

/**
 * The RouteWorker is responsible for implementing all non pub-sub logic of the RouteManager node
 * The class operates as a state machine with states LOADING_ROUTES, ROUTE_SELECTION, READY_TO_FOLLOW and FOLLOWING_ROUTE
 * The state of the RouteWorker is used by a ros node to determine which topics will be available
 */
public class RouteWorker {

  protected final IRouteManager routeManager;
  protected Route activeRoute;
  protected HashMap<String, Route> availableRoutes = new HashMap<>();
  protected double crossTrackDistance;
  protected RouteSegment currentSegment;
  protected Location hostVehicleLocation;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected final Log log;
  protected int currentWaypointIndex = 0;
  protected double downtrackDistance = 0;
  protected boolean systemOkay = false;
  protected boolean routeComplete = false;

  // State variables
  protected WorkerState state = WorkerState.LOADING_ROUTES;
  protected WorkerState[][] transitionTable = //Table controlling state transitions. row = event, col = currentState
    /*            STATES:         LOADING_ROUTES,          ROUTE_SELECTION,              READY_TO_FOLLOW               FOLLOWING_ROUTE */
    /*   EVENTS       */
    /*FILES_LOADED    */{{WorkerState.ROUTE_SELECTION, WorkerState.ROUTE_SELECTION, WorkerState.READY_TO_FOLLOW, WorkerState.FOLLOWING_ROUTE},
    /*ROUTE_SELECTED  */ {WorkerState.LOADING_ROUTES,  WorkerState.FOLLOWING_ROUTE, WorkerState.READY_TO_FOLLOW, WorkerState.FOLLOWING_ROUTE},
    /*ROUTE_COMPLETED */ {WorkerState.LOADING_ROUTES,  WorkerState.ROUTE_SELECTION, WorkerState.READY_TO_FOLLOW, WorkerState.ROUTE_SELECTION},
    /*LEFT_ROUTE      */ {WorkerState.LOADING_ROUTES,  WorkerState.ROUTE_SELECTION, WorkerState.READY_TO_FOLLOW, WorkerState.ROUTE_SELECTION},
    /*DRIVERS_READY    */ {WorkerState.LOADING_ROUTES,  WorkerState.ROUTE_SELECTION, WorkerState.FOLLOWING_ROUTE, WorkerState.FOLLOWING_ROUTE},
    /*SYSTEM_FAILURE  */ {WorkerState.LOADING_ROUTES,  WorkerState.ROUTE_SELECTION, WorkerState.READY_TO_FOLLOW, WorkerState.READY_TO_FOLLOW}};

  protected WorkerAction[][] actionTable = //Table controlling actions taken on state transitions. row = event, col = currentState
    /*        STATES:       LOADING_ROUTES,      ROUTE_SELECTION,       READY_TO_FOLLOW        FOLLOWING_ROUTE */
    /* EVENTS         */
    /*FILES_LOADED    */{{WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.NONE},
    /*ROUTE_SELECTED  */ {WorkerAction.INVALID, WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.CHANGE_ROUTE},
    /*ROUTE_COMPLETED */ {WorkerAction.INVALID, WorkerAction.INVALID, WorkerAction.INVALID, WorkerAction.MARK_COMPLETE},
    /*LEFT_ROUTE      */ {WorkerAction.INVALID, WorkerAction.INVALID, WorkerAction.INVALID, WorkerAction.LEFT_ROUTE_ALERT},
    /*DRIVERS_READY    */ {WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.MARK_SYSTEM_OK},
    /*SYSTEM_FAILURE  */ {WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.NONE,    WorkerAction.MARK_SYSTEM_NOT_OK}};

  /**
   * Constructor initializes a route worker object with the provided logging tool
   * @param log The logger to be used
   */
  public RouteWorker(IRouteManager manager, Log log) {
    this.log = log;
    this.routeManager = manager;
  }

  /**
   * Constructor initializes a route worker object with the provided logging tool and path to a route file directory
   * @param log The logger to be used
   * @param database_path The path to a directory of files compatible with the FileStrategy for loading routes
   */
  public RouteWorker(IRouteManager manager, Log log, String database_path){
    this.log = log;
    this.routeManager = manager;
    File folder = new File(database_path);
    File[] listOfFiles = folder.listFiles();

    for (int i = 0; i < listOfFiles.length; i++) {
      if (listOfFiles[i].isFile()) {
        FileStrategy loadStrategy = new FileStrategy(listOfFiles[i].getPath());
        loadAdditionalRoute(loadStrategy);
        handleStateTransition(WorkerEvent.FILES_LOADED);
      }
    }
  }

  /**
   * Function used to handle state transitions for a given event
   * @param event The event which triggered a need for a state transition
   */
  protected void handleStateTransition(WorkerEvent event) {
    handleAction(actionTable[event.ordinal()][state.ordinal()]);
    state = transitionTable[event.ordinal()][state.ordinal()];
  }

  /**
   * Function used to handle the execution of needed actions during a state transition
   * @param action The action to be executed
   */
  protected void handleAction(WorkerAction action){
    SystemAlert alertMsg;
    switch(action){
      case NONE:
        break;
      case CHANGE_ROUTE:
        // TODO: Recalculate downtrack distance and other metrics
        break;
      case MARK_COMPLETE:
        routeComplete = true;
        alertMsg = buildSystemAlertMsg(SystemAlert.CAUTION, "The end of the active route has been reached");
        routeManager.publishSystemAlert(alertMsg);
        break;
      case LEFT_ROUTE_ALERT:
        alertMsg = buildSystemAlertMsg(SystemAlert.WARNING, "Route: The host vehicle has left the route vicinity");
        routeManager.publishSystemAlert(alertMsg);
        break;
      case MARK_SYSTEM_OK:
        systemOkay = true;
        break;
      case MARK_SYSTEM_NOT_OK:
        systemOkay = false;
        break;
      case INVALID:
        String warning = "A state transition was attempted in the RouteWorker which was invalid";
        alertMsg = buildSystemAlertMsg(SystemAlert.WARNING, warning);
        routeManager.publishSystemAlert(alertMsg);
        log.error(warning);
        break;
      default:
        log.error("Unknown Route WorkerAction: " + action);
    }
  }

  /**
   * Gets the current state of the route worker
   * @return The state
   */
  public WorkerState getState() {
    return state;
  }

  /**
   * Returns true when the end of a route has been reached
   * @return the active route completion status
   */
  public boolean routeCompleted(){
    // TODO: implement using hostVehicleLocation
    return false;
  }

  /**
   * Loads a route into memory using the provided route loading strategy
   * @param loadStrategy
   */
  public void loadAdditionalRoute(IRouteLoadStrategy loadStrategy){
    Route route = loadStrategy.load();
    route.setRouteID(route.routeName); //TODO come up with better method of defining the route id
    availableRoutes.put(route.getRouteID(), route);
    handleStateTransition(WorkerEvent.FILES_LOADED);
  }

  //TODO: Implement
  /**
   * Returns true if crossTrackDistance is so large that the vehicle can no longer be considered on the route
   * @return vehicle on route status
   */
  private boolean leftRouteVicinity(){
    return false;
  }

  protected cav_srvs.GetAvailableRoutesResponse getAvailableRoutes() {
    List<cav_msgs.Route> routeMsgs = new LinkedList<>();
    cav_srvs.GetAvailableRoutesResponse response = messageFactory.newFromType(
      GetAvailableRoutesResponse._TYPE);

    System.out.println(availableRoutes);
    for (Route route : availableRoutes.values()){
      routeMsgs.add(route.toMessage(messageFactory));
    }
    response.setAvailableRoutes(routeMsgs);
    return response;
  }

  /**
   * Helper function which builds a system alert message
   * @param type The type of the alert
   * @param description Description of the message
   * @return System Alert message
   */
  protected SystemAlert buildSystemAlertMsg(byte type, String description) {
    SystemAlert alertMsg = messageFactory.newFromType(SystemAlert._TYPE);
    alertMsg.setType(type);
    alertMsg.setDescription(description);
    return alertMsg;
  }

  /**
   * Function to be used in a callback for the setActiveRoute service
   * @param request The service request
   * @return the service response
   */
  protected SetActiveRouteResponse setActiveRoute(SetActiveRouteRequest request) {
    SetActiveRouteResponse response = messageFactory.newFromType(SetActiveRouteResponse._TYPE);

    Route route = availableRoutes.get(request.getRouteID());
    // Check if the specified route exists.
    if (route == null) {
      response.setErrorStatus(SetActiveRouteResponse.NO_ROUTE);
    } else {
      activeRoute = route;
      //TODO Validation of ability to start route
      // Insert a starting waypoint at the current vehicle location which is connected to the route
      RouteWaypoint startingWP = new RouteWaypoint(hostVehicleLocation);
      activeRoute.insertWaypoint(startingWP, 0);

      response.setErrorStatus(SetActiveRouteResponse.NO_ERROR);
      handleStateTransition(WorkerEvent.ROUTE_SELECTED);
    }
    return response;
  }

  /**
   * Function to be used as a callback for the arrival of NavSatFix messages
   * @param msg The received message
   */
  protected void handleNavSatFixMsg(NavSatFix msg) {
    switch (msg.getStatus().getStatus()){
      case NavSatStatus.STATUS_NO_FIX:
        // TODO: Handle this in a way other than not updating the location
        log.warn("No gps data with fix received by route");
        break;
      case NavSatStatus.STATUS_FIX:
        hostVehicleLocation.setLocationData(msg.getLatitude(), msg.getLongitude(), msg.getAltitude());
        break;
      case NavSatStatus.STATUS_SBAS_FIX:
        //TODO: Handle this variant
        hostVehicleLocation.setLocationData(msg.getLatitude(), msg.getLongitude(), msg.getAltitude());
        break;
      case NavSatStatus.STATUS_GBAS_FIX:
        //TODO: Handle this variant
        hostVehicleLocation.setLocationData(msg.getLatitude(), msg.getLongitude(), msg.getAltitude());
        break;
      default:
        //TODO: Handle this variant maybe throw exception?
        log.error("Unknown nav sat fix status type: " + msg.getStatus().getStatus());
    }

    if (routeCompleted()) {
      handleStateTransition(WorkerEvent.ROUTE_COMPLETED);
    }

    if (leftRouteVicinity()) {
      handleStateTransition(WorkerEvent.LEFT_ROUTE);
    }
  }

  /**
   * Function to be used as a callback for received system alert messages
   * @param msg the system alert message
   */
  protected void handleSystemAlertMsg(SystemAlert msg) {
    switch (msg.getType()) {
      case cav_msgs.SystemAlert.CAUTION:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.WARNING:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.FATAL:
        handleStateTransition(WorkerEvent.SYSTEM_FAILURE);
        log.info("route_manager received system fatal on system_alert and is abandoning the route");
        break;
      case cav_msgs.SystemAlert.NOT_READY:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.DRIVERS_READY:
        handleStateTransition(WorkerEvent.DRIVERS_READY);
        log.info("route_manager received system ready on system_alert and is starting to publish");
        break;
      default:
        //TODO: Handle this variant maybe throw exception?
        log.error("System alert message received with unknown type: " + msg.getType());
    }
  }

  /**
   * Returns a message to be published on the current route segment topic
   * @return route segment message
   */
  protected cav_msgs.RouteSegment getCurrentRouteSegmentTopicMsg() {
    if (currentSegment == null)
      return messageFactory.newFromType(cav_msgs.RouteSegment._TYPE);
    return currentSegment.toMessage(messageFactory, currentWaypointIndex);
  }

  /**
   * Returns an active route message to publish
   * @return route message
   */
  protected cav_msgs.Route getActiveRouteTopicMsg() {
    if (activeRoute == null)
      return messageFactory.newFromType(cav_msgs.Route._TYPE);
    return activeRoute.toMessage(messageFactory);
  }

  /**
   * Returns a route state message to be published
   * @param seq The header sequence
   * @param time the time
   * @return route state message
   */
  protected RouteState getRouteStateTopicMsg(int seq, Time time) {
    if (activeRoute == null)
      return messageFactory.newFromType(cav_msgs.RouteState._TYPE);
    RouteState routeState = messageFactory.newFromType(RouteState._TYPE);
    routeState.setCrossTrack(crossTrackDistance);
    routeState.setRouteID(activeRoute.getRouteID());
    routeState.setDownTrack(downtrackDistance);

    std_msgs.Header hdr = messageFactory.newFromType(std_msgs.Header._TYPE);
    hdr.setFrameId("0");
    hdr.setSeq(seq);
    hdr.setStamp(time);
    return routeState;
  }

  /**
   * Function to be called in a repeating execution loop
   * @param sequenceNumber The sequence count of the loop
   */
  protected void onLoop(int sequenceNumber) {
    // If an active route has been selected then publish the route
    if (this.getState() == WorkerState.READY_TO_FOLLOW || this.getState() == WorkerState.FOLLOWING_ROUTE) {
      routeManager.publishActiveRoute(getActiveRouteTopicMsg());
    }

    // If following a selected route then publish the route state and current segment
    if (this.getState() == WorkerState.FOLLOWING_ROUTE){
      routeManager.publishRouteState(getRouteStateTopicMsg(sequenceNumber, routeManager.getTime()));
      routeManager.publishCurrentRouteSegment(getCurrentRouteSegmentTopicMsg());
    }
  }

  //  /**  TODO: Add once we have tim messages
  //   * Function for used in Tim topic callback. Used to update waypoints on a route
  //   * @param msg The tim message
  //   */
  //   void handleTimMsg(cav_msgs.Tim msg);
}
