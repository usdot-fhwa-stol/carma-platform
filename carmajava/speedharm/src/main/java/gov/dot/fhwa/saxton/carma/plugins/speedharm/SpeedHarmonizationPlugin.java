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
public class SpeedHarmonizationPlugin extends AbstractPlugin{
  protected String vehicleId = "";
  protected String serverUrl = "";
  protected boolean endSessionOnSuspend = true;
  protected int serverVehicleId = 0;
  protected long timestepDuration = 1000;
  protected RestTemplate restClient = new RestTemplate();
  protected Duration prevLatency = null;
  protected Duration measuredLatency = null;
  protected VehicleDataManager vehicleDataManager;
  protected LocalDateTime lastUpdateTime = LocalDateTime.now();

  public SpeedHarmonizationPlugin(PluginServiceLocator psl) {
    super(psl);
    version.setName("Speed Harmonization Plugin");
    version.setMajorRevision(1);
  }

  private void registerNewVehicleSession() {
    VehicleSession session = new VehicleSession();
    session.setUniqVehId(vehicleId);
    session.setDescription("USDOT CARMA Platform Vehicle with ID: " + vehicleId);
    log.info("Preparing to register vehicle: " + vehicleId + " with STOL infrastructure server:" + serverUrl + " ...");

    HttpHeaders postHeaders = new HttpHeaders();
    postHeaders.setContentType(MediaType.APPLICATION_JSON);
    HttpEntity<VehicleSession> postEntity = new HttpEntity<VehicleSession>(session, postHeaders);
    ResponseEntity<List<VehicleSession>> resp = restClient.exchange(serverUrl + UrlConstants.VEHICLE_LIST,
    HttpMethod.POST,
    postEntity,
    new ParameterizedTypeReference<List<VehicleSession>>(){});

    HttpHeaders responseHeaders = resp.getHeaders();
    String location = responseHeaders.getFirst("Location");
    String[] urlParts = location.split("/");
    serverVehicleId = Integer.parseInt(urlParts[urlParts.length - 1]);

    log.info("Registration successful, assigned vehicle session ID: " + serverVehicleId);
  }

	@Override
	public void onInitialize() {
    serverUrl = pluginServiceLocator.getParameterSource().getString("~infrastructure_server_url");
    vehicleId = pluginServiceLocator.getParameterSource().getString("~vehicle_id");
    double freq = pluginServiceLocator.getParameterSource().getDouble("~data_reporting_frequency");
    timestepDuration = (long) (1000.0 / freq);

    if (!endSessionOnSuspend) {
      registerNewVehicleSession();
    }
	}

	@Override
	public void onResume() {
    if (endSessionOnSuspend) {
      registerNewVehicleSession();
    }
	}

	@Override
	public void loop() throws InterruptedException {
    long timestepStart = System.currentTimeMillis();

    NetworkLatencyInformation latencyData = new NetworkLatencyInformation();
    latencyData.setVehicleTxTimestamp(LocalDateTime.now());
    latencyData.setVehicleMeasuredNetworkLatency(measuredLatency);

    VehicleStatusUpdate vsu = new VehicleStatusUpdate();
    vsu.setAutomatedControlState(vehicleDataManager.getAutomatedControl());
    vsu.setDistanceToNearestRadarObject(vehicleDataManager.getRange());
    vsu.setHeading(vehicleDataManager.getHeading());
    vsu.setLat(vehicleDataManager.getLongitude());
    vsu.setLon(vehicleDataManager.getLatitude());
    vsu.setRelativeSpeedOfNearestRadarObject(vehicleDataManager.getRangeRate());
    vsu.setSpeed(vehicleDataManager.getSpeed());
    vsu.setAccel(vehicleDataManager.getAccel());
    vsu.setNetworkLatencyInformation(latencyData);

    VehicleStatusUpdate response = new VehicleStatusUpdate();
    LocalDateTime sendTime = LocalDateTime.now();
    try {
      response = restClient.postForObject(serverUrl + STATUS_LIST + "/" + vehicleId, vsu, VehicleStatusUpdate.class);

      if (response == null) {
        log.warn("Infrastructure server rejected status update: " + vsu);
      }
    } catch (RestClientException rce) {
        log.warn("Infrastructure server rejected status update: " + vsu);
    }

    lastUpdateTime = LocalDateTime.now();
    prevLatency = measuredLatency;
    measuredLatency = Duration.between(sendTime, lastUpdateTime).dividedBy(2L);

    long prevMs = 0;
    if (prevLatency != null) {
      prevMs = prevLatency.toMillis();
    }

    long curMs = measuredLatency.toMillis();
    double diff = 0.0;
    if (prevMs > 0) {
      diff = 100 * (curMs - prevMs) / prevMs;
    }

    if (Math.abs(diff) > 20.0)  {
      log.warn(String.format("Large network jitter detected! Latency was %.02f and now is %.02f, delta = %.02f%%",
      prevMs,
      curMs,
      diff));
    } else {
      log.info(String.format("Latency was %.02f and now is %.02f, delta = %.02f%%",
      prevMs,
      curMs,
      diff));
    }

    long timestepEnd = System.currentTimeMillis();
    long sleepDuration = Math.max(timestepDuration - (timestepEnd - timestepStart), 0);
    log.info("Speed Harmonization timestep complete, sleeping for " + sleepDuration);
    Thread.sleep(sleepDuration);
  }
  
  private void endVehicleSession() {
    if (serverVehicleId > 0) {
      restClient.delete(serverUrl + VEHICLE_LIST + "/" + serverVehicleId);
      serverVehicleId = 0;
      prevLatency = null;
      measuredLatency = null;
      lastUpdateTime = LocalDateTime.now();
      log.info("Ended vehicle session #" + serverVehicleId);
    }
  }

	@Override
	public void onSuspend() {
    if (endSessionOnSuspend) {
      endVehicleSession();
    }
	}

	@Override
	public void onTerminate() {
    if (!endSessionOnSuspend) {
      endVehicleSession();
    }
  }
  
  @Override
  public void planTrajectory(Trajectory traj, double expectedStartSpeed) {
    // STUB
  }
}