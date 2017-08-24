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
 * The RouteWorker is responsible for implementing all ros agnostic logic of the Route package
 */
public class RouteWorker implements IRouteWorker {

  protected Route activeRoute;
  protected HashMap<String, Route> availableRoutes = new HashMap<>();
  protected double crossTrackDistance;
  protected RouteSegment currentSegment;

  protected Location hostVehicleLocation;

  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected Log log;

  protected RouteWorkerState state = RouteWorkerState.WAITING_FOR_AVAILABLE_ROUTES;

  protected int currentWaypointIndex = 0;

  protected double downtrackDistance = 0;
  protected final int alertQueueSize = 10;
  // TODO any time a system alert needs to be published it should be placed on the queue. If more than 10 messages the front element(oldest) will be popped and lost
  protected PriorityQueue<SystemAlert> systemAlertQueue;

  public RouteWorker(Log log) {
    this.log = log;
    initPriorityQueue();
  }

  public RouteWorker(Log log, String database_path){
    this.log = log;
    initPriorityQueue();
    File folder = new File(database_path);
    File[] listOfFiles = folder.listFiles();

    for (int i = 0; i < listOfFiles.length; i++) {
      if (listOfFiles[i].isFile()) {
        FileStrategy loadStrategy = new FileStrategy(listOfFiles[i].getPath());
        loadAdditionalRoute(loadStrategy);
        state = RouteWorkerState.WAITING_FOR_SYSTEM_START;
      }
    }
  }

  protected void initPriorityQueue(){
    systemAlertQueue = new PriorityQueue<>(alertQueueSize, new Comparator<SystemAlert>() {
      // TODO fix this since the numbers of the enum are not in a organized order !!!
      @Override public int compare(SystemAlert alert1, SystemAlert alert2) {
        if (alert1.getType() < alert2.getType()) {
          return -1;
        } else if (alert1.getType() > alert2.getType()) {
          return 1;
        } else {
          return 0;
        }
      }
    });
  }

  // TODO: Need to handle previous state to current state
  protected void handleStateTransition() {

  }

  /**
   * Handles a request for the available routes
   * @return
   */
  @Override public cav_srvs.GetAvailableRoutesResponse getAvailableRoutes() {
    List<cav_msgs.Route> routeMsgs = new LinkedList<>();
    cav_srvs.GetAvailableRoutesResponse response = messageFactory.newFromType(
      GetAvailableRoutesResponse._TYPE);

    for (Route route : availableRoutes.values()){
      routeMsgs.add(route.toMessage(messageFactory));
    }
    response.setAvailableRoutes(routeMsgs);
    return response;
  }

  @Override public SetActiveRouteResponse setActiveRoute(SetActiveRouteRequest request) {
    SetActiveRouteResponse response = messageFactory.newFromType(SetActiveRouteResponse._TYPE);

    Route route = availableRoutes.get(request.getRouteID());
    // Check if the specified route exists.
    if (route == null) {
      response.setErrorStatus(SetActiveRouteResponse.NO_ROUTE);
    }
    activeRoute = route;
    response.setErrorStatus(SetActiveRouteResponse.NO_ERROR);
    state = RouteWorkerState.FOLLOWING_ROUTE;
    return response;
  }

  @Override public void handleNavSatFixMsg(NavSatFix msg) {
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
        log.error("Unknown nav sat fix status type Type = " + msg.getStatus().getStatus());
    }
  }

  @Override public void handleSystemAlertMsg(SystemAlert msg) {
    switch (msg.getType()) {
      case cav_msgs.SystemAlert.CAUTION:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.WARNING:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.FATAL:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.NOT_READY:
        // TODO: Handle this message type
        break;
      case cav_msgs.SystemAlert.SYSTEM_READY:
        state = RouteWorkerState.WAITING_FOR_ROUTE_SELECTION;
        log.info("route_manager received system ready on system_alert and is starting to publish");
        break;
      default:
        //TODO: Handle this variant maybe throw exception?
        log.error("System alert message received with unknown type = " + msg.getType());
    }
  }

  @Override public PriorityQueue<SystemAlert> getSystemAlertTopicMsgs() {
    return systemAlertQueue;
  }

  @Override public cav_msgs.RouteSegment getCurrentRouteSegmentTopicMsg() {
    return currentSegment.toMessage(messageFactory, currentWaypointIndex);
  }

  @Override public cav_msgs.Route getActiveRouteTopicMsg() {
    return activeRoute.toMessage(messageFactory);
  }

  @Override public RouteState getRouteStateTopicMsg(int seq, Time time) {
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

  public void loadAdditionalRoute(IRouteLoadStrategy loadStrategy){
    Route route = loadStrategy.load();
    availableRoutes.put(route.getRouteID(), route);
  }

  @Override public boolean isSystemStarted() {
    return systemStarted;
  }
}
