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

import org.springframework.core.ParameterizedTypeReference;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import gov.dot.fhwa.saxton.speedharm.api.objects.NetworkLatencyInformation;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleSession;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate.AutomatedControlStatus;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

import static gov.dot.fhwa.saxton.carma.plugins.speedharm.UrlConstants.*;

/**
 * Plugin implementing integration withe STOL I TO 22 Infrastructure Server
 * <p>
 * Commmunicates via the internet with the Infrastructure Server to report vehicle
 * state and receive speed commands as may relate to whatever algorithm the server
 * is configured to run with.
 */
public class SpeedHarmonizationPlugin extends AbstractPlugin {
  protected String vehicleId = "";
  protected String serverUrl = "";
  protected boolean endSessionOnSuspend = true;
  protected int serverVehicleId = 0;
  protected long timestepDuration = 1000;

  protected StatusUpdater statusUpdater = null;
  protected Thread statusUpdaterThread = null;

  protected CommandReceiver commandReceiver = null;
  protected Thread commandReceiverThread = null;

  protected SessionManager sessionManager;
  protected VehicleDataManager vehicleDataManager;
  protected LocalDateTime lastUpdateTime = LocalDateTime.now();

  public SpeedHarmonizationPlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Speed Harmonization Plugin");
    version.setMajorRevision(1);
  }

  @Override
  public void onInitialize() {
    serverUrl = pluginServiceLocator.getParameterSource().getString("~infrastructure_server_url");
    vehicleId = pluginServiceLocator.getParameterSource().getString("~vehicle_id");
    double freq = pluginServiceLocator.getParameterSource().getDouble("~data_reporting_frequency");
    timestepDuration = (long) (1000.0 / freq);

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
      statusUpdater = new StatusUpdater(serverUrl, sessionManager.getServerSessionId(),  new RestTemplate() , timestepDuration, vehicleDataManager);
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
    // NO-OP } 
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

  @Override
  public void planTrajectory(Trajectory traj, double expectedStartSpeed) {
    // STUB
  }
}