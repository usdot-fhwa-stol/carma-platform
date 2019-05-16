/*
 * Copyright (C) 2018-2019 LEIDOS.
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
import gov.dot.fhwa.saxton.carma.rosutils.MobilityHelper;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.FileStrategy;
import gov.dot.fhwa.saxton.carma.route.Route;
import gov.dot.fhwa.saxton.carma.route.RouteSegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.UnaryOperator;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.rosjava_geometry.Transform;

import cav_msgs.BSM;
import cav_msgs.BSMCoreData;
import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
 
/**
 * Primary logic class for the RSUMeterManager node
 * 
 * This class handles tracking of platoons and vehicles on the road, but passes the communications and control logic to the current state object
 */
public class RSUMeterWorker {

  protected final static double MS_PER_S = 1000.0; // Milli-seconds per second
  protected final static long NANO_SEC_PER_MS = 1000000L; // Nano-seconds per milli-second
  protected final static long BSM_ID_TIMEOUT = 3000L; // Timeout of a bsm id in ms
  protected final static long PLATOON_TIMEOUT = 4000L; // Timeout of a platooning info message
  protected static final double MIN_PLATOON_SPEED = 0.5; // m/s - could be more intelligent about comparing these to host's current speed
  protected static final double MAX_PLATOON_SPEED = 35.0; // m/s
  protected final IRSUMeterManager manager;
  protected final SaxtonLogger log;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected final Route mainRoadRoute;
  protected final double distToMerge;
  protected final double mainRouteMergeDTD;
  protected final Location meterLoc;
  protected final Point3D meterECEF;
  protected final int targetLane;
  protected final double mergeLength;
  protected final String BROADCAST_ID = "";
  protected final static String PLATOONING_STRATEGY = "Carma/Platooning";
  protected final static String COOPERATIVE_MERGE_STRATEGY = "Carma/CooperativeMerge";
  protected final String PLATOON_INFO_PARAMS = "INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d"; // Sent every three seconds 
  protected final String INFO_TYPE_PARAM = "INFO";
  protected final List<String> INFO_STRATEGY_PARAMS = new ArrayList<>(Arrays.asList("REAR", "LENGTH", "SPEED", "SIZE"));
  
  protected final ConcurrentMap<String, PlatoonData> platoonMap = new ConcurrentHashMap<>();
  protected final ConcurrentMap<String, BSM> bsmMap = new ConcurrentHashMap<>();
  protected final Object stateMutex = new Object();
  protected AtomicReference<IRSUMeteringState> state = new AtomicReference<>(); // Never null
  protected final String rsuId;
  protected String platoonRearBSMId = null;
  protected final GeodesicCartesianConverter gcc =  new GeodesicCartesianConverter();

  protected final double meterRadius;
  protected final double minApproachAccel;
  protected final double targetApproachSpeed;

  protected final long timeMargin;
  protected final long requestPeriod;
  protected final long commandPeriod;
  protected final long commsTimeout;

  protected final double driverLagTime;
  protected final double commsLagTime;
  

  /**
   * Constructor
   * 
   * @param manager IRSUMeterManager responsible for providing timing and publishing capabilities
   * @param log Logging object
   * @param routeFilePath The file path of the route which defines the main road (not the on ramp)
   * @param rsuId The static id of this rsu
   * @param distToMerge The distance in m from the ramp meter point to the start of the merge
   * @param mainRouteMergeDTD The distance along the main road route in m of the start of the merge
   * @param meterRadius The radius around the merge point where the rsu can take control of a vehicle
   * @param targetLane The lane id which a merge is into
   * @param mergeLength The length in m of the merge area
   * @param timeMargin The allowable time margin for a merge operation
   * @param requestPeriod The period of meter request broadcasts
   * @param commandPeriod The period of commands sent to a controlled vehicles 
   * @param commsTimeout The time period of a comms timeout between the rsu and the vehicle
   * @param meterLoc The location of the meter point on the earth
   * @param minApproachAccel The minimum acceleration of the vehicle when approaching the meter point which ensures comfort based on controller behavior. Always Positive
   * @param targetApproachSpeed The speed which the vehicle will have right as it rolls to the meter point stop bar. Should be just above crawl speed
   * @param driverLagTime The lag time in seconds for the driver to hit the accelerator
   * @param commsLagTime The lag time in seconds for the communications between a platoon -> rsu -> merge vehicle
   */
  RSUMeterWorker(IRSUMeterManager manager, SaxtonLogger log, String routeFilePath,
    String rsuId, double distToMerge, double mainRouteMergeDTD, double meterRadius,
    int targetLane, double mergeLength, long timeMargin,
    long requestPeriod, long commandPeriod, long commsTimeout, Location meterLoc,
    double minApproachAccel, double targetApproachSpeed, double driverLagTime, double commsLagTime) throws IllegalArgumentException {
    
    this.manager = manager;
    this.log = log;
    this.distToMerge = distToMerge;
    this.mainRouteMergeDTD = mainRouteMergeDTD;
    this.meterRadius = meterRadius;
    this.targetLane = targetLane;
    this.mergeLength = mergeLength;
    this.rsuId = rsuId;
    this.timeMargin = timeMargin;
    this.requestPeriod = requestPeriod;
    this.commandPeriod = commandPeriod;
    this.commsTimeout = commsTimeout;
    this.meterLoc = meterLoc;
    this.meterECEF = gcc.geodesic2Cartesian(meterLoc, Transform.identity());
    this.minApproachAccel = minApproachAccel;
    this.targetApproachSpeed = targetApproachSpeed;
    this.driverLagTime = driverLagTime;
    this.commsLagTime = commsLagTime;

    // Load route file
    log.info("RouteFile: " + routeFilePath);

    FileStrategy loadStrategy = new FileStrategy(routeFilePath, log.getBaseLoggerObject());
    Route loadedRoute = loadStrategy.load(); // Load route

    if (loadedRoute == null) {
      throw new IllegalArgumentException("Failed to load the main road route file");
    }

    loadedRoute.setRouteID(loadedRoute.getRouteName()); // Set route id

    mainRoadRoute = Route.fromMessage(loadedRoute.toMessage(messageFactory)); // Assign waypoint ids

    // Set state this is required to maintain the state not null contract
    state.set(new StandbyState(this, log));
  }

  /**
   * Handles incoming BSM messages
   * BSM messages are cached for later reference
   * 
   * @param msg The BSM message to process
   */
  public void handleBSMMsg(BSM msg) {

    String bsmId = bsmIdFromBuffer(msg.getCoreData().getId());

    if (bsmId == null) {
      log.warn("Null BSM Id ");
      return;
    }

    double lat = msg.getCoreData().getLatitude();
    double lon = msg.getCoreData().getLongitude();
    double alt = msg.getCoreData().getElev();
    
    Location loc = new Location(lat,lon,alt);
    if (!bsmMap.containsKey(bsmId)) {
      log.debug("New BSM Id: " + bsmId + " lat: " + lat + " lon: " + lon + " elev: " + alt);
    }
    bsmMap.put(bsmId, msg);
  }

  /**
   * Helper function to convert a channel buffer from a bsm message id into a hex string
   * 
   * @param buffer The bsm id bytes to convert
   * 
   * @return The hex string of this bsm id 
   */
  private String bsmIdFromBuffer(ChannelBuffer buffer) {

    int capacity =  buffer.capacity();

    if (capacity != 4) {
      log.warn("Tried to process bsm id of less than 4 bytes: " + buffer);
      return null;
    }

    byte[] idArray = new byte[4];
    for(int i = 0; i < capacity; i++) {
      idArray[i] = buffer.getByte(i);
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
   * Handles MobilityOperation messages
   * 
   * This function will cache platooning information and pass the messages onto the current state
   * 
   * @param msg The message to handle
   */
  public void handleMobilityOperationMsg(MobilityOperation msg) {
    // Validate message is for us
    if (!(msg.getHeader().getRecipientId().equals(rsuId) 
      || msg.getHeader().getRecipientId().equals(BROADCAST_ID))) {
      return;
    }

    // If this message is an info broadcast from a platoon, update platoon info
    if (msg.getStrategy().equals(PLATOONING_STRATEGY) 
      && msg.getStrategyParams().contains(INFO_TYPE_PARAM)) {

      updatePlatoonWithOperationMsg(msg);
    }

    // Pass on to state
    state.get().onMobilityOperationMessage(msg);

  }

  /**
   * Updates the cached platooning information based on the provided message
   * 
   * @param msg The platooning message to process
   */
  private void updatePlatoonWithOperationMsg(MobilityOperation msg) {

    String strategyParams = msg.getStrategyParams();
    List<String> paramsArray;
    try {
      paramsArray = MobilityHelper.extractStrategyParams(strategyParams, INFO_TYPE_PARAM, INFO_STRATEGY_PARAMS);
    } catch(IllegalArgumentException e) {
      log.warn("Bad operations strategy string received. Generated exception: " + e);
      return;
    }
    
    double platoonSpeed = Double.parseDouble(paramsArray.get(2));
    String rearBsmId = paramsArray.get(0);

    //perform sanity check on received params - no way to check BSM ID
    if (platoonSpeed < MIN_PLATOON_SPEED  ||  platoonSpeed > MAX_PLATOON_SPEED) {
      log.warn("Received operation message with suspect strategy values. platoonSpeed = " + platoonSpeed);
      return;
    }

    BSM cachedMsg = bsmMap.get(rearBsmId);
    // If we don't have a BSM for this rear vehicle then no value in tracking platoon
    if (cachedMsg == null) {
      log.warn("Platoon detected before BSM data available. Expected BSM Id: " + rearBsmId);
      return;
    }

    BSMCoreData bsmData = cachedMsg.getCoreData();

    Location rearLoc = new Location(bsmData.getLatitude(), bsmData.getLongitude(), bsmData.getElev());


    // Get downtrack distance of platoon rear
    double platoonRearDTD = getDowntrackDistanceFromLocation(rearLoc);

    // If the platoon is passed the end of the merge region, we don't need to track it any more
    if (platoonRearDTD > mainRouteMergeDTD + mergeLength) {
      PlatoonData removedPlatoon = platoonMap.remove(msg.getHeader().getSenderId());
      log.debug("Platoon removed as it is past the merge point. " + removedPlatoon);
      return;
    }

    // At this point the platoon still needs to be tracked

    // For this calculation we assume constant speed
    double distanceLeft = mainRouteMergeDTD - platoonRearDTD;
    double deltaT = distanceLeft / platoonSpeed;
    long timeOfArrival = (long)(deltaT * MS_PER_S)  + msg.getHeader().getTimestamp();
    
    PlatoonData newData = new PlatoonData(msg.getHeader().getSenderId(), platoonRearDTD,
     platoonSpeed, timeOfArrival, rearBsmId, System.currentTimeMillis());
     
    log.debug("Platoon added " + newData);
    platoonMap.put(newData.getLeaderId(), newData);
  }

  /**
   * Handles a MobilityRequest message
   * 
   * Requests are passed to the current state and the acceptance or rejection is broadcast back to the sending vehicle
   * 
   * @param msg The message to process
   */
  public void handleMobilityRequestMsg(MobilityRequest msg) {
    // Validate message is for us
    // We ignore broadcast request messages as well for this algorithm
    if (!msg.getHeader().getRecipientId().equals(rsuId)) {
        return;
      }

    boolean accepted = state.get().onMobilityRequestMessage(msg);
    
    MobilityResponse response = messageFactory.newFromType(MobilityResponse._TYPE);
    response.getHeader().setRecipientId(msg.getHeader().getSenderId());
    response.getHeader().setSenderId(rsuId);
    response.getHeader().setTimestamp(System.currentTimeMillis());
    response.getHeader().setPlanId(msg.getHeader().getPlanId());
    response.setIsAccepted(accepted);

    manager.publishMobilityResponse(response);
  }

  /**
   * Handle a MobilityResponse message
   * 
   * Messages are passed onto the current state
   * 
   * @param msg The message to be processed
   */
  public void handleMobilityResponseMsg(MobilityResponse msg) {
    // Validate message is for us
    if (!(msg.getHeader().getRecipientId().equals(rsuId) 
      || msg.getHeader().getRecipientId().equals(BROADCAST_ID))) {
        return;
      }

    state.get().onMobilityResponseMessage(msg);
  }

  /**
   * Helper function converts a location param string value into downtrack distance on a route
   * 
   * @param loc The location to map to a downtrack value
   * 
   * @return the downtrack distance
   */
  private double getDowntrackDistanceFromLocation(Location loc) {
    Point3D ecefPoint = gcc.geodesic2Cartesian(loc, Transform.identity());

    RouteSegment seg = mainRoadRoute.routeSegmentOfPoint(ecefPoint, mainRoadRoute.getSegments()); // TODO could be optimized
    double segmentDowntrack = seg.downTrackDistance(ecefPoint);
    return segmentDowntrack + mainRoadRoute.lengthOfSegments(0, seg.getUptrackWaypoint().getWaypointId() - 1);
  }

  /**
   * Sets the state of this plugin
   * The contact here is that the old state is always the state requesting the transition
   * This function is thread safe as long as the contract is met
   * 
   * @param callingState The state which is requesting the transition
   * @param newState The state to transition to
   */
  protected void setState(IRSUMeteringState callingState, IRSUMeteringState newState) {

    // Only perform the transition if we are still in the expected calling state
    if (state.compareAndSet(callingState, newState)) {
      log.info("Transitioned from old state: " + callingState + " to new state: " + newState); 
    }
  }

  /**
   * Returns the expected travel time for a vehicle with the provided inputs
   * 
   * Time calculated as a speed up from current speed to max speed followed by a steady speed to completion
   * 
   * @param dist The distance to travel in m
   * @param speed The current speed in m/s
   * @param maxSpeed The max allowed speed in m/s
   * @param lagTime The response time of the vehicle in s
   * @param maxAccel The maximum acceleration capabilities of the vehicle
   * 
   * @return the expected travel time in ms
   */
  protected long expectedTravelTime(double dist, double speed, double maxSpeed, double lagTime, double maxAccel) {
    // If the speed is at or above max
    if (speed > maxSpeed - 0.1) {
      double deltaT = dist / speed;
      return (long)(deltaT * MS_PER_S);
    } 
    
    // Account for speed up

    double deltaV = (maxSpeed - speed);
    double timeToSpeedUp = (deltaV / maxAccel);



    double distCoveredInSpeedUp = 0.5 * deltaV * timeToSpeedUp + speed * timeToSpeedUp;
    double distRemaining = dist - distCoveredInSpeedUp;

    double timeAtMaxSpeed = distRemaining / maxSpeed;

    double totalTime = timeToSpeedUp + timeAtMaxSpeed;


    return (long)((totalTime + lagTime) * MS_PER_S);
  }

  public PlatoonData getNextPlatoon(String ignoredPlatoon) {
    PlatoonData mostRecentPlatoon = null;

    // Find the platoon with the earliest time of arrival
    // Use of .forEach is avoided to ensure sequential execution
    for (PlatoonData platoon : platoonMap.values()) {
      if (platoon.getLeaderId().equals(ignoredPlatoon)) {
        continue;
      }
      if (mostRecentPlatoon == null) {
        mostRecentPlatoon = platoon;
      } else if (platoon.getExpectedTimeOfArrival() < mostRecentPlatoon.getExpectedTimeOfArrival()
        && platoon.getRearDTD() < mainRouteMergeDTD + mergeLength) { 
        mostRecentPlatoon = platoon;
      }
    }

    return mostRecentPlatoon;
  }

  /**
   * A loop which will spin as fast as possible
   */
  public void loop() throws InterruptedException {
    state.get().loop();
    removeOldBSMS();
    removeOldPlatoons();
  }

  /**
   * Helper function removes platoon data from platoons which are no longer sending updates
   */
  private void removeOldPlatoons() {
    // Remove the bsm if it's id has expired
    final long cutoffTime = System.currentTimeMillis() - PLATOON_TIMEOUT;
    
    platoonMap.values().removeIf(platoon -> {
      return platoon.getStamp() < cutoffTime;
    });
  }

  /**
   * Helper function which removes bsms which are no longer sending updates
   */
  private void removeOldBSMS() {
    // Remove the bsm if it's id has expired
    // Since the header stamp is set by us not the sender there is no need to synchronize clocks
    final Time cutoffTime = Time.fromMillis(System.currentTimeMillis() - BSM_ID_TIMEOUT);
    
    bsmMap.values().removeIf(bsmMsg -> {
      return bsmMsg.getHeader().getStamp().compareTo(cutoffTime) < 0;
    });
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
   * @return the distToMerge
   */
  public double getDistToMerge() {
    return distToMerge;
  }

  /**
   * @return the meterRadius
   */
  public double getMeterRadius() {
    return meterRadius;
  }

  /**
   * @return the targetLane
   */
  public int getTargetLane() {
    return targetLane;
  }

  /**
   * @return the mergeLength
   */
  public double getMergeLength() {
    return mergeLength;
  }

  /**
   * @return the manager
   */
  public IRSUMeterManager getManager() {
    return manager;
  }

  /**
   * @return the timeMargin
   */
  public long getTimeMargin() {
    return timeMargin;
  }

  /**
   * @return the commandPeriod
   */
  public long getCommandPeriod() {
    return commandPeriod;
  }

  /**
   * @return the requestPeriod
   */
  public long getRequestPeriod() {
    return requestPeriod;
  }

  /**
   * @return the commsTimeout
   */
  public long getCommsTimeout() {
    return commsTimeout;
  }

  /**
   * @return the meterLoc
   */
  public Location getMeterLoc() {
    return meterLoc;
  }

  /**
   * @return the meterECEF
   */
  public Point3D getMeterECEF() {
    return meterECEF;
  }

  /**
   * @return the minApproachAccel
   */
  public double getMinApproachAccel() {
    return minApproachAccel;
  }

  /**
   * @return the targetApproachSpeed
   */
  public double getTargetApproachSpeed() {
    return targetApproachSpeed;
  }

  /**
   * @return the driverLagTime
   */
  public double getDriverLagTime() {
    return driverLagTime;
  }

  /**
   * @return the commsLagTime
   */
  public double getCommsLagTime() {
    return commsLagTime;
  }
}
