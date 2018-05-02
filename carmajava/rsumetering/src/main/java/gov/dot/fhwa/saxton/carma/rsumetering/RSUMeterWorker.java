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

package gov.dot.fhwa.saxton.carma.rsumetering;

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import gov.dot.fhwa.saxton.carma.rsumetering.IRSUMeteringState;
import gov.dot.fhwa.saxton.carma.rsumetering.PlatoonData;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import cav_msgs.BSM;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
 
/**
 * Primary logic class for the RSUMeterManager node
 */
public class RSUMeterWorker {

  protected final static double MS_PER_S = 1000.0; // Milli-seconds per second
  protected final IRSUMeterManager manager;
  protected final SaxtonLogger log;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected final Route mainRoadRoute;
  protected final double distToMerg;
  protected final double mergeDTD;
  protected final String BROADCAST_ID = "";
  protected final static String PLATOONING_STRATEGY = "Carma/Platooning";
  protected final static String COOPERATIVE_MERGE_STRATEGY = "Carma/CooperativeMerge";
  protected final String PLATOON_INFO_PARAMS = "INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d"; // Sent every three seconds (TODO make faster)
  protected final String INFO_TYPE_PARAM = "INFO";
  protected final List<String> INFO_STRATEGY_PARAMS = new ArrayList<>(Arrays.asList("REAR", "LENGTH", "SPEED", "SIZE"));
  

  protected final int mergeWpId;
  protected final ConcurrentMap<String, PlatoonData> platoonMap = new ConcurrentHashMap<>();
  protected final ConcurrentMap<String, Location> bsmMap = new ConcurrentHashMap<>();
  protected final Object stateMutex = new Object();
  protected IRSUMeteringState state = null;
  protected final String rsuId = "RSU_ID"; // TODO this should be read from param
  protected String platoonRearBSMId = null;
  protected final GeodesicCartesianConverter gcc =  new GeodesicCartesianConverter();

  protected final double meterRadius;
  

  /**
   * Constructor
   * 
   * @param manager IRSUMeterManager responsible for providing timing and publishing capabilities
   * @param log Logging object
   * @param routeFilePath
   */
  RSUMeterWorker(IRSUMeterManager manager, SaxtonLogger log, String routeFilePath,
   double distToMerg, int mergeWpId, double meterRadius) throws IllegalArgumentException {
    this.manager = manager;
    this.log = log;
    this.distToMerg = distToMerg;
    this.mergeWpId = mergeWpId;
    this.meterRadius = meterRadius;

    // Load route file
    log.info("RouteFile: " + routeFilePath);

    FileStrategy loadStrategy = new FileStrategy(routeFilePath, log.getBaseLoggerObject());
    Route loadedRoute = loadStrategy.load(); // Load route

    if (loadedRoute == null) {
      throw new IllegalArgumentException("Failed to load the main road route file");
    }

    loadedRoute.setRouteID(loadedRoute.getRouteName()); // Set route id

    mainRoadRoute = Route.fromMessage(loadedRoute.toMessage(messageFactory)); // Assign waypoint ids

    this.mergeDTD = mainRoadRoute.lengthOfSegments(0, mergeWpId);
  }

  public void handleBSMMsg(BSM msg) {

    String bsmId = bsmIdFromBuffer(msg.getCoreData().getId());

    boolean[] foundBSM = new boolean[1];

    platoonMap.forEach(
      (k, v) -> {
        if (v.getRearBSMId().equals(bsmId)) {
          foundBSM[0] = true;
        }
      }
    );

    // If this bsm is relevant for a tracked platoon store the location
    if (foundBSM[0]) {
      double lat = msg.getCoreData().getLatitude();
      double lon = msg.getCoreData().getLongitude();
      double alt = msg.getCoreData().getElev();
  
      Location loc = new Location(lat,lon,alt);
      bsmMap.put(bsmId, loc);
    }
  }

  private String bsmIdFromBuffer(ChannelBuffer buffer) {
    byte[] idArray = buffer.array();

    if (idArray.length != 4) {
      log.warn("Tried to process bsm id of less than 4 bytes: " + idArray);
      return null;
    }

    // Convert bsm id array to string
    char[] hexChars = new char[idArray.length * 2];
    for(int i = 0; i < idArray.length; i++) {
        int firstFourBits = (0xF0 & idArray[i]) >>> 4;
        int lastFourBits = 0xF & idArray[i];
        hexChars[i * 2] = Integer.toHexString(firstFourBits).charAt(0);
        hexChars[i * 2 + 1] = Integer.toHexString(lastFourBits).charAt(0);
    }
    return new String(hexChars);
  }

  /**
   * Handles control messages. 
   * If the requested axleAngle is greater than the angleThreshold then it will be considered a request for a turn.
   * The UI will then be notified. 
   */
  public void handleMobilityOperationMsg(MobilityOperation msg) {
    // TODO need check to distinguish between platoon message and cooperative merge message
    // TODO need full validation check as we will not be using the mobility router
    // Check if this message is a broadcast info message from a platoon. If not return
    if (!msg.getHeader().getRecipientId().equals(BROADCAST_ID) 
      || !msg.getStrategy().equals(PLATOONING_STRATEGY) 
      || !msg.getStrategyParams().contains(INFO_TYPE_PARAM)) {
      return;
    }

    String strategyParams = msg.getStrategyParams();
    List<String> paramsArray;
    try {
      paramsArray = extractStrategyParams(strategyParams, INFO_TYPE_PARAM, INFO_STRATEGY_PARAMS);
    } catch(IllegalArgumentException e) {
      log.warn("Bad operations strategy string received. Generated exception: " + e);
      return;
    }
    
    double platoonSpeed = Double.parseDouble(paramsArray.get(2)); // TODO assign index to constant
    String rearBsmId = paramsArray.get(0);
    Location rearLoc = bsmMap.get(rearBsmId);
    // If we don't have a BSM for this rear vehicle add an incomplete set of data so we will grab the bsm later
    if (rearLoc == null) {
      // TODO think about robustness
      PlatoonData newData = new PlatoonData(msg.getHeader().getSenderId(), 0, platoonSpeed, 0, rearBsmId);
      platoonMap.put(newData.getLeaderId(), newData);
      return;
    }


    // Get downtrack distance of platoon rear
    double platoonRearDTD = getDowntrackDistanceFromLocation(rearLoc);


    // For this calculation we assume constant speed limit
    double distanceLeft = mergeDTD - platoonRearDTD;
    double deltaT = distanceLeft / platoonSpeed;
    long timeOfArrival = (long)(deltaT * MS_PER_S)  + msg.getHeader().getTimestamp();
    
    PlatoonData newData = new PlatoonData(msg.getHeader().getSenderId(), platoonRearDTD,
     platoonSpeed, timeOfArrival, rearBsmId);
     
    platoonMap.put(newData.getLeaderId(), newData);
    // TODO we can drop the platoon if it has passed the merge point
  }

  // TODO
  public void handleMobilityRequestMsg(MobilityRequest msg) {
    synchronized(stateMutex) {
      if (state == null) {
        log.warn("Requested to handle mobility request message but state was null");
      }
      boolean accepted = state.onMobilityRequestMessage(msg);
      
      MobilityResponse response = messageFactory.newFromType(MobilityResponse._TYPE);
      response.getHeader().setRecipientId(msg.getHeader().getSenderId());
      response.getHeader().setSenderId(rsuId);
      response.getHeader().setTimestamp(System.currentTimeMillis());
      response.getHeader().setPlanId(msg.getHeader().getPlanId());
      response.setIsAccepted(accepted);

      manager.publishMobilityResponse(response);
    }
  }

  public void handleMobilityResponseMsg(MobilityRequest msg) {
    synchronized(stateMutex) {
      if (state == null) {
        log.warn("Requested to handle mobility request message but state was null");
      }
      state.onMobilityRequestMessage(msg);
    }
  }


  protected List<String> extractStrategyParams(String paramsString, String expectedType, List<String> keys) 
    throws IllegalArgumentException {

    String dataString = paramsString;

    // If we expect a data type extract and validate it
    if (expectedType != null) {
      String[] paramsParts  = paramsString.split("\\|");

      // Check the correct type string was provided
      if (paramsParts.length != 2
        || !paramsParts[0].equals(expectedType)) {
        throw new IllegalArgumentException("Invalid type. Expected: " + expectedType + " String: " + paramsString);
      }

      dataString = paramsParts[1]; // Get data string
    }

    Pattern pattern = Pattern.compile("(?<=(^|,))(.*?)(?=(,|$))"); // Reg ex grabs everything between two commas or the start and end of a string
    Matcher matcher = pattern.matcher(dataString);

    List<String> dataList = new ArrayList<>(keys.size());

    // Iterate expected number of times and extract values
    for (int i = 0; i < keys.size(); i++) {
      // If we can't extract a value the input string is badly formatted
      if (!matcher.find()) {
        throw new IllegalArgumentException("Failed to find pattern match between commas. String: " + paramsString);
      }

      String[] dataParts = matcher.group().split(":");

      // Check if the key is correct
      if (dataParts.length != 2
       || !dataParts[0].equals(keys.get(i))) {
        throw new IllegalArgumentException("Invalid key. Expected: " + keys.get(i) + " String: " + paramsString);
      }

      dataList.add(dataParts[1]);
    }

    return dataList;
  }

  /**
   * Helper function converts a location param string value into downtrack distance on a route
   * 
   * @param loc The location to map to a downtrack value
   */
  private double getDowntrackDistanceFromLocation(Location loc) {
    Point3D ecefPoint = gcc.geodesic2Cartesian(loc, Transform.identity());

    RouteSegment seg = mainRoadRoute.routeSegmentOfPoint(ecefPoint, mainRoadRoute.getSegments()); // TODO could be optimized
    double segmentDowntrack = seg.downTrackDistance(ecefPoint);
    return segmentDowntrack + mainRoadRoute.lengthOfSegments(0, seg.getUptrackWaypoint().getWaypointId() - 1);
  }

  /**
   * Sets the state of this plugin
   * 
   * @param newState The state to transition to
   */
  protected void setState(IRSUMeteringState newState) {
    log.info("Transitioned from old state: " + state + " to new state: " + newState);
    synchronized (stateMutex) {
      state = newState;
    }
  }

  protected long expectedTravelTime(double dist, double speed, double maxSpeed, double lagTime, double maxAccel) {

  // TODO we need to handle the case of speed > maxSpeed throw exception or something
    // If the speed is at max
    if (Math.abs(maxSpeed - speed) < 0.1) {
      double deltaT = dist / speed;
      return (long)(deltaT * MS_PER_S);
    } 
    
    // Account for speed up

    double timeToSpeedUp = ((maxSpeed - speed) / maxAccel) + lagTime;

    double distCoveredInSpeedUp = 0.5 * (maxSpeed + speed) * timeToSpeedUp;
    double distRemaining = dist - distCoveredInSpeedUp;

    double timeAtMaxSpeed = distRemaining / maxSpeed;

    double totalTime = timeToSpeedUp + timeAtMaxSpeed;


    return (long)(totalTime * MS_PER_S);
  }

  public void publishMobilityOperation(MobilityOperation msg) {
    manager.publishMobilityOperation(msg);
  }


  // TODO using this would have limitations with multiple platoons
  public PlatoonData getNextPlatoon() {
    PlatoonData[] mostRecentPlatoon = new PlatoonData[1];

    // Find the platoon with the earliest time of arrival
    platoonMap.forEach(
      (k, v) -> {
        if (mostRecentPlatoon[0] == null) {
          mostRecentPlatoon[0] = v;
        } else if (v.getExpectedTimeOfArrival() < mostRecentPlatoon[0].getExpectedTimeOfArrival()) {
          mostRecentPlatoon[0] = v;
        }
      }
    );

    return mostRecentPlatoon[0];
  }


  /**
   * @return the mainRoadRoute
   */
  public Route getMainRoadRoute() {
    return mainRoadRoute;
  }

  /**
   * @return the rsuId
   */
  public String getRsuId() {
    return rsuId;
  }

  /**
   * @return the distToMergOnRamp
   */
  public double getDistToMerg() {
    return distToMerg;
  }

  /**
   * @return the meterRadius
   */
  public double getMeterRadius() {
    return meterRadius;
  }
}
