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

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.speedharm.api.objects.NetworkLatencyInformation;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleStatusUpdate;

import java.time.Duration;
import java.time.LocalDateTime;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;
import static gov.dot.fhwa.saxton.carma.plugins.speedharm.UrlConstants.*;

/**
 * Handles the communications necessary to send status updates to the 
 * STOL infrastructure server
 */
public class StatusUpdater implements Runnable {
  protected ILogger log = LoggerManager.getLogger();
  protected String serverUrl = "";
  protected long timestepDuration;
  protected VehicleDataManager vehicleDataManager;
  protected RestTemplate restClient;
  protected int vehicleId;
  protected LocalDateTime lastUpdateTime = LocalDateTime.now();
  protected Duration prevLatency = null;
  protected Duration measuredLatency = null;

  StatusUpdater(String serverUrl, int vehicleId, RestTemplate restClient, long timestepDuration,
      VehicleDataManager vehicleDataManager) {
    this.serverUrl = serverUrl;
    this.vehicleId = vehicleId;
    this.restClient = restClient;
    this.timestepDuration = timestepDuration;
    this.vehicleDataManager = vehicleDataManager;
  }

  @Override
  public void run() {
    while (!Thread.currentThread().isInterrupted()) {
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
        } else {
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

          if (Math.abs(diff) > 20.0) {
            log.warn(String.format("Large network jitter detected! Latency was %.02f and now is %.02f, delta = %.02f%%",
                prevMs, curMs, diff));
          } else {
            log.info(String.format("Latency was %.02f and now is %.02f, delta = %.02f%%", prevMs, curMs, diff));
          }

        }
      } catch (RestClientException rce) {
        log.warn("Infrastructure server rejected status update: " + vsu);
      }

      long timestepEnd = System.currentTimeMillis();
      long sleepDuration = Math.max(timestepDuration - (timestepEnd - timestepStart), 0);
      log.info("Speed Harmonization timestep complete, sleeping for " + sleepDuration);

      try {
        Thread.sleep(sleepDuration);
      } catch (InterruptedException e) {
        break;
      }
    }
  }
}
