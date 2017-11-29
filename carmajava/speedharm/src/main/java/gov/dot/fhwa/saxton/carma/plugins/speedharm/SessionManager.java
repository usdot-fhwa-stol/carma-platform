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
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleSession;
import java.util.List;
import org.springframework.core.ParameterizedTypeReference;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.client.RestTemplate;

/** * Handles the communications necessary to create and destroy vehicle sessions on the
 * STOL infrastructure server
 */
public class SessionManager {
  protected String serverUrl;
  protected String vehicleId;
  protected RestTemplate restClient;
  protected int serverVehicleId = 0;

  public SessionManager(String serverUrl, String vehicleId, RestTemplate restClient) {
    this.serverUrl = serverUrl;
    this.vehicleId = vehicleId;
    this.restClient = restClient;
  }

  protected ILogger log = LoggerManager.getLogger();

  public void registerNewVehicleSession() {
    if (serverVehicleId > 0) {
      log.warn("Attempted vehicle registration while session still open.");
      return;
    }

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

  public int getServerSessionId() {
    return serverVehicleId;
  }

  public void endVehicleSession() {
    if (serverVehicleId > 0) {
      restClient.delete(serverUrl + UrlConstants.VEHICLE_LIST + "/" + serverVehicleId);
      serverVehicleId = 0;
      log.info("Ended vehicle session #" + serverVehicleId);
    }
  }
}
