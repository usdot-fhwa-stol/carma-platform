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
import org.springframework.web.client.RestTemplate;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.AccStrategyManager;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.carma.guidance.util.AlgorithmFlags;
import java.time.LocalDateTime;
import java.util.List;
import java.util.SortedSet;

/**
 * Plugin implementing integration withe STOL I TO 22 Infrastructure Server
 * <p>
 * Commmunicates via the internet with the Infrastructure Server to report vehicle
 * state and receive speed commands as may relate to whatever algorithm the server
 * is configured to run with.
 */
public class SpeedHarmonizationPlugin extends AbstractPlugin implements ISpeedHarmInputs {
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

  private static final String SPEED_HARM_FLAG = "SPEEDHARM";

  public SpeedHarmonizationPlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Speed Harmonization Plugin");
    version.setMajorRevision(1);
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

    vehicleDataManager = new VehicleDataManager();
    vehicleDataManager.init();

    sessionManager = new SessionManager(serverUrl, vehicleId, new RestTemplate());
    if (!endSessionOnSuspend) {
      sessionManager.registerNewVehicleSession();
    }
  }

  @Override
  public void onResume() {
    if (endSessionOnSuspend) {
      sessionManager.registerNewVehicleSession();
    }

    if (statusUpdaterThread == null && statusUpdater == null) {
      statusUpdater = new StatusUpdater(serverUrl, sessionManager.getServerSessionId(), new RestTemplate(),
          timestepDuration, vehicleDataManager);
      statusUpdaterThread = new Thread(statusUpdater);
      statusUpdaterThread.setName("SpeedHarm Status Updater");
      statusUpdaterThread.start();
    }

    if (commandReceiverThread == null && commandReceiver == null) {
      commandReceiver = new CommandReceiver(serverUrl, sessionManager.getServerSessionId(), new RestTemplate());
      commandReceiverThread = new Thread(commandReceiver);
      statusUpdaterThread.setName("SpeedHarm Command Receiver");
      commandReceiverThread.start();
    }
  }

  @Override
  public void loop() throws InterruptedException {
    // We don't actually do anything on the loop thread, just set a sufficiently high sleep value
    Thread.sleep(10000);
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

    if (endSessionOnSuspend) {
      sessionManager.endVehicleSession();
    }
  }

  @Override
  public void onTerminate() {
    if (!endSessionOnSuspend) {
      sessionManager.endVehicleSession();
    }
  }

  private void planComplexManeuver(Trajectory traj, double start, double end) {
    SpeedHarmonizationManeuver maneuver = new SpeedHarmonizationManeuver(
    this, 
    pluginServiceLocator.getManeuverPlanner().getManeuverInputs(), 
    pluginServiceLocator.getManeuverPlanner().getGuidanceCommands(), 
    AccStrategyManager.newAccStrategy(), 
    start, 
    end, 
    1.0,  // Dummy values for now. TODO: Replace
    100.0);

    traj.setComplexManeuver(maneuver);
  }

  @Override
  public void planTrajectory(Trajectory traj, double expectedStartSpeed) {
    List<IManeuver> maneuvers = traj.getManeuvers();
    double complexManeuverStartLocation = -1.0;
    if (!maneuvers.isEmpty()) {
      // Get the location of the last maneuver in the list
      complexManeuverStartLocation = maneuvers.get(maneuvers.size() - 1).getEndDistance();
    } else {
      // Fill the whole trajectory if legal
      complexManeuverStartLocation = traj.getStartLocation();
    }

    // Find the earliest window after the start location at which speedharm is enabled
    SortedSet<AlgorithmFlags> flags = pluginServiceLocator.getRouteService()
        .getAlgorithmFlagsInRange(complexManeuverStartLocation, traj.getEndLocation());

    double earliestLegalWindow = complexManeuverStartLocation;
    double endOfWindow = complexManeuverStartLocation;
    for (AlgorithmFlags flagset : flags) {
      if (!flagset.getDisabledAlgorithms().contains(SPEED_HARM_FLAG)) {
        earliestLegalWindow = flagset.getLocation();
        break;
      }
    }

    // Find the end of that same window
    for (AlgorithmFlags flagset : flags) {
      if (flagset.getLocation() > earliestLegalWindow && flagset.getDisabledAlgorithms().contains(SPEED_HARM_FLAG)) {
        endOfWindow = flagset.getLocation();
        break;
      }
    }

    if (Math.abs(endOfWindow - earliestLegalWindow) > minimumManeuverLength) {
      planComplexManeuver(traj, earliestLegalWindow, endOfWindow);
    }
  }

  @Override
  public double getSpeedCommand() {
    return commandReceiver.getLastCommand().getSpeed();
  }

  @Override
  public double getMaxAccelLimit() {
    return Math.min(Math.abs(maxAccel), 2.5);
  }

  @Override
  public Duration getTimeSinceLastUpdate() {
    LocalDateTime now = LocalDateTime.now();
    long millis = java.time.Duration.between(commandReceiver.getLastCommand().getTimestamp(), now).toMillis();
    return Duration.fromMillis(millis);
  }
}