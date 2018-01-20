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

package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import org.ros.message.Duration;
import org.springframework.http.converter.HttpMessageConverter;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * Plugin implementing integration withe STOL I TO 22 Infrastructure Server
 * <p>
 * Commmunicates via the internet with the Infrastructure Server to report vehicle
 * state and receive speed commands as may relate to whatever algorithm the server
 * is configured to run with.
 */
public class SpeedHarmonizationPlugin extends AbstractPlugin implements ISpeedHarmInputs, IStrategicPlugin {
  protected String vehicleId = "";
  protected String serverUrl = "";
  protected boolean endSessionOnSuspend = true;
  protected int serverVehicleId = 0;
  protected long timestepDuration = 1000;
  protected double minimumManeuverLength = 10.0;
  protected double maxAccel = 2.0;

  protected StatusUpdater statusUpdater = null;
  protected Thread statusUpdaterThread = null;

  protected CommandReceiver commandReceiver = null;
  protected Thread commandReceiverThread = null;

  protected SessionManager sessionManager;
  protected VehicleDataManager vehicleDataManager;
  protected LocalDateTime lastUpdateTime = LocalDateTime.now();
  protected RestTemplate restClient;

  private static final String SPEED_HARM_FLAG = "SPEEDHARM";

  public SpeedHarmonizationPlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Speed Harmonization Plugin");
    version.setMajorRevision(1);

    restClient = new RestTemplate();
  }

  @Override
  public void onInitialize() {
    serverUrl = pluginServiceLocator.getParameterSource().getString("~infrastructure_server_url",
        "http://localhost:5000");
    vehicleId = pluginServiceLocator.getParameterSource().getString("~vehicle_id");
    double freq = pluginServiceLocator.getParameterSource().getDouble("~data_reporting_frequency", 1.0);
    timestepDuration = (long) (1000.0 / freq);
    minimumManeuverLength = pluginServiceLocator.getParameterSource().getDouble("~speed_harm_min_maneuver_length",
        10.0);
    maxAccel = pluginServiceLocator.getParameterSource().getDouble("~speed_harm_max_accel", 2.0);

    List<HttpMessageConverter<?>> httpMappers = new ArrayList<HttpMessageConverter<?>>();
    MappingJackson2HttpMessageConverter jsonMapper = new MappingJackson2HttpMessageConverter();
    jsonMapper.getObjectMapper().findAndRegisterModules();
    httpMappers.add(jsonMapper);
    restClient.setMessageConverters(httpMappers);

    vehicleDataManager = new VehicleDataManager();
    vehicleDataManager.init(pubSubService);

    sessionManager = new SessionManager(serverUrl, vehicleId, restClient);
  }

  @Override
  public void onResume() {
    // Loop until we connect to the server
    while (true) {
      try {
        sessionManager.registerNewVehicleSession();
        break;
      } catch (RestClientException rce) {
        log.warn("Unable to register with server, caught exception.", rce);
        try {
          Thread.sleep(100); // So we dont spin super-tight
        } catch (InterruptedException ie) {
          Thread.currentThread().interrupt();
        }
      }
    }

    if (statusUpdaterThread == null && statusUpdater == null) {
      statusUpdater = new StatusUpdater(serverUrl, sessionManager.getServerSessionId(), restClient, timestepDuration,
          vehicleDataManager);
      statusUpdaterThread = new Thread(statusUpdater);
      statusUpdaterThread.setName("SpeedHarm Status Updater");
      statusUpdaterThread.start();
    }

    if (commandReceiverThread == null && commandReceiver == null) {
      commandReceiver = new CommandReceiver(serverUrl, sessionManager.getServerSessionId(), restClient);
      commandReceiverThread = new Thread(commandReceiver);
      commandReceiverThread.setName("SpeedHarm Command Receiver");
      commandReceiverThread.start();
    }
  }

  @Override
  public void loop() throws InterruptedException {
    long tsStart = System.currentTimeMillis();
    if (statusUpdater.lastUpdateTime != null) {
      // If we've successfully communicated with the server recently, signal our availability
      java.time.Duration timeSinceLastUpdate = java.time.Duration.between(statusUpdater.lastUpdateTime,
          LocalDateTime.now());
      if (timeSinceLastUpdate.toMillis() < 3 * timestepDuration) {
        setAvailability(true);
      } else {
        setAvailability(false);
      }
    }

    vehicleDataManager.setManeuverRunning(pluginServiceLocator.getArbitratorService()
        .getCurrentlyExecutingManeuver(ManeuverType.COMPLEX) instanceof SpeedHarmonizationManeuver);

    long tsEnd = System.currentTimeMillis();
    long sleepDuration = Math.max(100 - (tsEnd - tsStart), 0);
    Thread.sleep(sleepDuration);
  }

  @Override
  public void onSuspend() {
    if (statusUpdaterThread != null && statusUpdater != null) {
      statusUpdaterThread.interrupt();
      statusUpdaterThread = null;
      statusUpdater = null;
    }

    if (commandReceiverThread != null && commandReceiver != null) {
      commandReceiverThread.interrupt();
      commandReceiverThread = null;
      commandReceiver = null;
    }

    sessionManager.endVehicleSession();
  }

  @Override
  public void onTerminate() {
    // NO-OP
  }

  private void planComplexManeuver(Trajectory traj, double start, double end) {
    SpeedHarmonizationManeuver maneuver = new SpeedHarmonizationManeuver(this,
        pluginServiceLocator.getManeuverPlanner().getManeuverInputs(),
        pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(), AccStrategyManager.newAccStrategy(), start,
        end, 1.0, // Dummy values for now. TODO: Replace
        100.0);

    boolean accepted = traj.setComplexManeuver(maneuver);
    log.info("Trajectory response to complex maneuver = " + accepted);
  }

  @Override
  public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedStartSpeed) {
    List<IManeuver> maneuvers = traj.getManeuvers();
    double complexManeuverStartLocation = -1.0;
    if (!maneuvers.isEmpty()) {
      // Get the location of the last maneuver in the list
      complexManeuverStartLocation = maneuvers.get(maneuvers.size() - 1).getEndDistance();
      log.info("planTrajectory(): IF: complexManeuverStartLocation = " + complexManeuverStartLocation);
    } else {
      // Fill the whole trajectory if legal
      complexManeuverStartLocation = traj.getStartLocation();
      log.info("planTrajectory(): ELSE: complexManeuverStartLocation = " + complexManeuverStartLocation);
    }

    // Find the earliest window after the start location at which speedharm is enabled
    double[] planWindow = pluginServiceLocator.getRouteService().getAlgorithmEnabledWindowInRange(
            complexManeuverStartLocation, traj.getEndLocation(), SPEED_HARM_FLAG);
    if (planWindow == null) {
      log.info("Couldn't find plan window at start: " + complexManeuverStartLocation + " endOfWindow: " + traj.getEndLocation());
      return new TrajectoryPlanningResponse();
    }

    if (Math.abs(planWindow[1] - planWindow[0]) > minimumManeuverLength) {
      planComplexManeuver(traj, planWindow[0], planWindow[1]);
    } else {
      log.warn("Unable to find sufficient window to plan Speed Harmonization maneuver");
    }

    return new TrajectoryPlanningResponse();
  }

  @Override
  public double getSpeedCommand() {
    if (commandReceiver.getLastCommand() != null) {
      log.info("Using received command");
      return commandReceiver.getLastCommand().getSpeed();
    } else {
      log.info("Using previous vehicle speed");
      return vehicleDataManager.getSpeed();
    }
  }

  @Override
  public double getMaxAccelLimit() {
    return Math.min(Math.abs(maxAccel), 2.5);
  }

  @Override
  public Duration getTimeSinceLastUpdate() {
    if (commandReceiver != null && commandReceiver.getLastCommand() != null) {
      LocalDateTime now = LocalDateTime.now();
      long millis = java.time.Duration.between(commandReceiver.getLastCommand().getTimestamp(), now).toMillis();
      return Duration.fromMillis(millis);
    } else {
      return Duration.fromMillis(0);
    }
  }
}