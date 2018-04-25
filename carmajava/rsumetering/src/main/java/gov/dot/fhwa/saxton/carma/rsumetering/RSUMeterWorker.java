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

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import gov.dot.fhwa.saxton.carma.route.Route;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import cav_msgs.MobilityOperation;
 
/**
 * Primary logic class for the RSUMeterManager node
 */
public class RSUMeterWorker {

  protected IRSUMeterManager manager;
  protected SaxtonLogger log;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
  protected Route mainRoadRoute;
  protected double distToMergOnRamp;
  protected String targetId;
  protected double mergeDTD;
  protected final String BROADCAST_ID = "";
  protected final String PLATOONING_STRATEGY = "Carma/Platooning";
  protected final String INFO_STRATEGY_PARAM = "INFO";
  //"INFO|LEADER:%s,REAR_DTD:%.2f,SPEED:%.2f,SIZE:%d" // Sent every 3s
  protected final String REAR_LOCATION_STRATEGY_PARAM = "REAR_DTD";
  protected final String SPEED_STRATEGY_PARAM = "SPEED";
  protected final String SIZE_STRATEGY_PARAM = "SIZE";

  /**
   * Constructor
   * 
   * @param manager IRSUMeterManager responsible for providing timing and publishing capabilities
   * @param log Logging object
   * @param routeFilePath
   */
  RSUMeterWorker(IRSUMeterManager manager, SaxtonLogger log, String routeFilePath,
   double distToMergOnRamp, String targetId, int mergeWPId) {
    this.manager = manager;
    this.log = log;
    this.distToMergOnRamp = distToMergOnRamp;
    this.targetId = targetId;
    this.mergeWPId = mergeWPId;

    // Load route file
    log.info("RouteFile: " + routeFilePath);

    FileStrategy loadStrategy = new FileStrategy(routeFilePath, log);
    this.mainRoadRoute = loadStrategy.load(); // Load route

    if (route == null) {
      throw new Exception("Failed to load the main road route file");
    }

    mainRoadRoute.setRouteID(route.getRouteName()); // Set route id

    mainRoadRoute = Route.fromMessage(mainRoadRoute.toMessage()); // Assign waypoint ids
  }

  /**
   * Handles control messages. 
   * If the requested axleAngle is greater than the angleThreshold then it will be considered a request for a turn.
   * The UI will then be notified. 
   */
  public void handleMobilityOperationMsg(MobilityOperation msg) {
    // Check if this message is a broadcast info message from a platoon. If not return
    if (!msg.getHeader().getRecipientId().equals(BROADCAST_ID) 
      || !msg.getStrategy().equals(PLATOONING_STRATEGY) 
      || !msg.getStrategyParams().contains(INFO_STRATEGY_PARAM)) {
      return;
    }

    String strategyParams = msg.getStrategyParams();
    String[] paramsArray = splitStrategyParams(strategyParams);
    double platoonRearDTD = getDowntrackDistanceFromLocation(paramsArray[1]); // TODO assign index to constant
    double platoonSpeed = Double.parseDouble(paramsArray[2]); // TODO assign index to constant

    // TODO calculate the time till platoon passes the merge point
    // TODO we can precalculate the time to go from our hold location to the merge
    // TODO Wait for the difference between the two times to pass then send the request message
  }

  /**
   * TODO: Helper function splits a param string and returns and array of param values
   * 
   * @param strategyParams The strategy params string from a mobility operation message
   */
  private String[] splitStrategyParams(String strategyParams) {
    return new String[4];
  }

  /**
   * TODO: Helper function converts a location param string value into downtrack distance on a route
   * 
   * @param platoonRearLocation The strategy params string which will be converted
   */
  private double getDowntrackDistanceFromLocation(String platoonRearLocation) {
    return 0;
  }
}
